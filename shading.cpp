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
using namespace std::placeholders;

float light_contribution(const vec3& normal, float albedo, const Light& light) {
    return light.intensity * albedo * glm::max(0.f,glm::dot(normal,-light.trans_dir)) * glm::one_over_pi<float>();
}

unsigned char shade(const vec3& normal,float albedo, const vector<Light> lights) {
    //determine colour of pixel given barycentric coordinates

    vector<float> light_contributions(lights.size());
    auto lc = std::bind(light_contribution,normal,albedo,_1);
    std::transform(lights.begin(),lights.end(),light_contributions.begin(), lc);
    
    float result = glm::min(std::accumulate(light_contributions.begin(),light_contributions.end(),0.f),255.f);

    return (unsigned char)result;
}