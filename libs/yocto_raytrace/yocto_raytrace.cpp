//
// Implementation for Yocto/RayTrace.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2021 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "yocto_raytrace.h"

#include <yocto/yocto_cli.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shading.h>
#include <yocto/yocto_shape.h>

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const camera_data& camera, const vec2f& uv) {
  auto film = camera.aspect >= 1
                  ? vec2f{camera.film, camera.film / camera.aspect}
                  : vec2f{camera.film * camera.aspect, camera.film};
  auto q    = transform_point(camera.frame,
      {film.x * (0.5f - uv.x), film.y * (uv.y - 0.5f), camera.lens});
  auto e    = transform_point(camera.frame, {0, 0, 0});
  return {e, normalize(e - q)};
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

// Raytrace renderer.
static vec4f shade_raytrace(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    auto env = eval_environment(scene, ray.d);
    return {env.x, env.y, env.z, 1};
  }
  auto& instance = scene.instances[isec.instance];
  auto& shape    = scene.shapes[instance.shape];
  auto& material = eval_material(scene, instance, isec.element, isec.uv);
  auto  normal   = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  auto color = material.color;

  if (rand1f(rng) < 1 - material.opacity) {
    return shade_raytrace(
        scene, bvh, ray3f{position, ray.d}, bounce + 1, rng, params);
  }

  auto radiance = material.emission;
  if (bounce >= params.bounces) return {radiance.x, radiance.y, radiance.z, 1};

  auto outgoing = -ray.d;

  if (!shape.points.empty()) {
    normal = -ray.d;
  } else if (!shape.lines.empty()) {
    normal = orthonormalize(-ray.d, normal);
  } else if (!shape.triangles.empty()) {
    if (dot(-ray.d, normal) < 0) {
      normal = -normal;
    }
  }

  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
      radiance += color *
                  xyz(shade_raytrace(scene, bvh, ray3f{position, incoming},
                      bounce + 1, rng, params));
      break;
    }
    case material_type::reflective: {
      auto mnormal = normal;
      if (material.roughness > 0) {
        mnormal = sample_hemisphere_cospower(
            2 / pow(material.roughness, 2), normal, rand2f(rng));
      }
      auto incoming = reflect(outgoing, mnormal);
      radiance += color *
                  xyz(shade_raytrace(scene, bvh, ray3f{position, incoming},
                      bounce + 1, rng, params));
      break;
    }
    case material_type::glossy: {
      auto mnormal = sample_hemisphere_cospower(
          2 / pow(material.roughness, 2), normal, rand2f(rng));
      auto rand = vec3f{rand1f(rng)};
      auto f    = fresnel_schlick(vec3f{0.04}, mnormal, outgoing);
      if (rand.x < f.x && rand.y < f.y && rand.z < f.z) {
        auto incoming = reflect(outgoing, mnormal);
        radiance += xyz(shade_raytrace(
            scene, bvh, ray3f{position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
        radiance += color *
                    xyz(shade_raytrace(scene, bvh, ray3f{position, incoming},
                        bounce + 1, rng, params));
      }
      break;
    }
    case material_type::transparent: {
      auto rand = vec3f{rand1f(rng)};
      auto f    = fresnel_schlick(vec3f{0.04}, normal, outgoing);
      if (rand.x < f.x && rand.y < f.y && rand.z < f.z) {
        auto incoming = reflect(outgoing, normal);
        radiance += xyz(shade_raytrace(
            scene, bvh, ray3f{position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = -outgoing;
        radiance += color *
                    xyz(shade_raytrace(scene, bvh, ray3f{position, incoming},
                        bounce + 1, rng, params));
      }
      break;
    }
    case material_type::refractive: {
      vec3f  incoming;
      auto   entering       = dot(outgoing, normal) >= 0;
      auto   up_normal      = entering ? normal : -normal;
      auto   rel_ior        = entering ? (1.0f / material.ior) : material.ior;
      double cos_theta      = fmin(dot(-outgoing, up_normal), 1.0);
      double sin_theta      = sqrt(1.0 - cos_theta * cos_theta);
      bool   cannot_refract = rel_ior * sin_theta > 1.0;

      if (cannot_refract ||
          rand1f(rng) >
              fresnel_schlick(vec3f{rel_ior}, up_normal, outgoing).x) {
        incoming = reflect(outgoing, up_normal);
      } else {
        incoming = refract(outgoing, up_normal, rel_ior);
      }

      radiance += color *
                  xyz(shade_raytrace(scene, bvh, ray3f{position, incoming},
                      bounce + 1, rng, params));
      break;
    }
    case material_type::volumetric: {
      // material.density = vec3f{0.5, 0.5, 0.5};
      material.density = vec3f{0.99, 0.99, 0.99};
      auto isec2       = intersect_bvh(
          bvh, scene, isec.instance, ray3f{position, ray.d});

      float max_distance;
      vec3f p1;

      if (isec2.hit) {  // ero fuori dall'istanza
        p1 = transform_point(
            instance.frame, eval_position(shape, isec2.element, isec2.uv));
        max_distance = distance(p1, position);
        p1 -= ray.d * ray.tmin;  // fix bug intersezioni con pavimento
      } else {                   // ero interno all'istanza
        p1           = position;
        position     = ray.o;
        max_distance = isec.distance;
      }

      auto probab = eval_transmittance(
          material.density, max_distance);  // calcolo probabilità scattering

      if (fmod(rand1f(rng), 1) + 0.05f > sum(probab) / 3) {  // se scattera
        auto  distance = sample_transmittance(  // calcolo distanza scattering
            material.density, max_distance, rand1f(rng), rand1f(rng));
        vec3f origin   = position + distance * ray.d;

        auto min_x = min(position.x, p1.x);
        auto max_x = max(position.x, p1.x);
        auto min_y = min(position.y, p1.y);
        auto max_y = max(position.y, p1.y);
        auto min_z = min(position.z, p1.z);
        auto max_z = max(position.z, p1.z);

        origin = vec3f{// fix float moltiplication bug
            clamp(origin.x, min_x + ray.tmin * 10, max_x - ray.tmin * 10),
            clamp(origin.y, min_y + ray.tmin * 10, max_y - ray.tmin * 10),
            clamp(origin.z, min_z + ray.tmin * 10, max_z - ray.tmin * 10)};

        vec3f incoming;
        while (true) {
          incoming = vec3f{fmod(rand1f(rng), 2.0f) - 1,
              fmod(rand1f(rng), 2.0f) - 1, fmod(rand1f(rng), 2.0f) - 1};
          if (pow(incoming.x, 2) + pow(incoming.y, 2) + pow(incoming.z, 2) >=
              1) {
            continue;
          }
          break;
        }

        auto isec3 = intersect_bvh(
            bvh, scene, isec.instance, ray3f{origin, incoming});
        vec3f p2 = origin;  // fallback per i casi speciali
        if (isec3.hit) {
          p2 = transform_point(
              instance.frame, eval_position(shape, isec3.element, isec3.uv));
        }
        // else {
        //  if (!isec2.hit
        //      /* se era 2d o ero dentro all'istanza */
        //      || max_distance / 2 < ray.tmin
        //      /* se ero troppo vicino al bordo */) {
        //  } else {
        //    /*printf("\nprobab: %f %f %f | %d \n", probab.x, probab.y,
        //    probab.z, isec2.hit); printf("p0: %f %f %f | p1: %f %f %f \n",
        //    position.x, position.y, position.z, p1.x, p1.y, p1.z); printf(
        //        "origin: %f %f %f | ray.d: %f %f %f | dist: %f | max_dist: %f
        //    \n ", origin.x, origin.y, origin.z, ray.d.x, ray.d.y, ray.d.z,
        //        distance, max_distance);
        //    printf("QUESTO NON DOVREBBE MAI ACCADERE\n");*/
        //  }
        //}
        //}
        p2 -= ray.d * ray.tmin;  // fix bug intersezioni con pavimento
        radiance += color * xyz(shade_raytrace(scene, bvh, ray3f{p2, incoming},
                                bounce + 1, rng, params));
      } else {  // se non scattero, proietto oltre l'istanza
        radiance += xyz(shade_raytrace(
            scene, bvh, ray3f{p1, ray.d}, bounce + 1, rng, params));
      }
      break;
    }
  }
  return {radiance.x, radiance.y, radiance.z, 1};
}

// Matte renderer.
static vec4f shade_matte(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // YOUR CODE GOES HERE ----
  return {0, 0, 0, 0};
}

// Eyelight renderer.
static vec4f shade_eyelight(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) return {0, 0, 0, 0};
  auto& instance = scene.instances[isec.instance];
  auto& shape    = scene.shapes[instance.shape];
  auto& material = scene.materials[instance.material];
  auto  normal   = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  auto color = material.color * dot(normal, -ray.d);
  return vec4f{color.x, color.y, color.z, 1};
}

static vec4f shade_normal(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) return {0, 0, 0, 0};
  auto& instance = scene.instances[isec.instance];
  auto& shape    = scene.shapes[instance.shape];
  auto  normal   = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  normal = normal * 0.5 + 0.5;
  return vec4f{normal.x, normal.y, normal.z, 1};
}

static vec4f shade_texcoord(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) return {0, 0, 0, 0};
  auto& instance = scene.instances[isec.instance];
  auto& shape    = scene.shapes[instance.shape];
  auto  coord    = eval_texcoord(shape, isec.element, isec.uv);
  return {fmod(coord.x, 1), fmod(coord.y, 1), 0, 1};
}

static vec4f shade_color(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) return {0, 0, 0, 0};
  auto& instance = scene.instances[isec.instance];
  auto  color    = scene.materials[instance.material].color;
  return vec4f{color.x, color.y, color.z, 1};
}

// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const scene_data& scene,
    const bvh_scene& bvh, const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;
    case raytrace_shader_type::matte: return shade_matte;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Build the bvh acceleration structure.
bvh_scene make_bvh(const scene_data& scene, const raytrace_params& params) {
  return make_bvh(scene, false, false, params.noparallel);
}

// Init a sequence of random number generators.
raytrace_state make_state(
    const scene_data& scene, const raytrace_params& params) {
  auto& camera = scene.cameras[params.camera];
  auto  state  = raytrace_state{};
  if (camera.aspect >= 1) {
    state.width  = params.resolution;
    state.height = (int)round(params.resolution / camera.aspect);
  } else {
    state.height = params.resolution;
    state.width  = (int)round(params.resolution * camera.aspect);
  }
  state.samples = 0;
  state.image.assign(state.width * state.height, {0, 0, 0, 0});
  state.hits.assign(state.width * state.height, 0);
  state.rngs.assign(state.width * state.height, {});
  auto rng_ = make_rng(1301081);
  for (auto& rng : state.rngs) {
    rng = make_rng(961748941ull, rand1i(rng_, 1 << 31) / 2 + 1);
  }
  return state;
}

// Progressively compute an image by calling trace_samples multiple times.
void raytrace_samples(raytrace_state& state, const scene_data& scene,
    const bvh_scene& bvh, const raytrace_params& params) {
  if (state.samples >= params.samples) return;
  auto& camera = scene.cameras[params.camera];
  auto  shader = get_shader(params);
  state.samples += 1;
  if (params.samples == 1) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u = (i + 0.5f) / state.width, v = (j + 0.5f) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else if (params.noparallel) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else {
    parallel_for(state.width * state.height, [&](int idx) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    });
  }
}

// Check image type
static void check_image(
    const color_image& image, int width, int height, bool linear) {
  if (image.width != width || image.height != height)
    throw std::invalid_argument{"image should have the same size"};
  if (image.linear != linear)
    throw std::invalid_argument{
        linear ? "expected linear image" : "expected srgb image"};
}

// Get resulting render
color_image get_render(const raytrace_state& state) {
  auto image = make_image(state.width, state.height, true);
  get_render(image, state);
  return image;
}
void get_render(color_image& image, const raytrace_state& state) {
  check_image(image, state.width, state.height, true);
  auto scale = 1.0f / (float)state.samples;
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    image.pixels[idx] = state.image[idx] * scale;
  }
}

}  // namespace yocto
