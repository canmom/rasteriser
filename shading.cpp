#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>

#include <glm/glm.hpp>
#include <glm/vec3.hpp>

#include <glm/gtc/constants.hpp>

#include "light.h"

#include "shading.h"

using std::vector;
using glm::vec3;
using glm::uvec3;
using namespace std::placeholders;

vec3 light_contribution(const vec3& normal, const vec3& albedo, const Light& light) {
    return light.intensity * light.colour * albedo * glm::max(0.f,glm::dot(normal,-light.trans_dir)) * glm::one_over_pi<float>();
}

uvec3 shade(const vec3& normal, const vec3& albedo, const vector<Light> lights) {
    //determine colour of pixel given barycentric coordinates

    vector<vec3> light_contributions(lights.size());
    auto lc = std::bind(light_contribution,normal,albedo,_1);
    std::transform(lights.begin(),lights.end(),light_contributions.begin(), lc);
    
    vec3 result = glm::min(std::accumulate(light_contributions.begin(),light_contributions.end(),vec3(0.f)),vec3(255.f));

    return (uvec3)result;
}