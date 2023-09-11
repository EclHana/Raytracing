#pragma once

#include "Scene.hpp"
#include <atomic>
#include <mutex>

struct hit_payload
{
    float tNear;
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
	Renderer() = default;

    void Render(const Scene& scene);
    std::atomic<int> cnt;
    std::mutex mtx;
};