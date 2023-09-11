#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "Global.hpp"
#include <chrono>

int main(int argc, char **argv)
{
    Scene scene(784, 784);

    Material *red = new Material(MaterialType::diffuse, Vector3f());
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material *green = new Material(MaterialType::diffuse, Vector3f());
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material *white = new Material(MaterialType::diffuse, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material *light = new Material(MaterialType::diffuse, (8.0f * Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
                                                           15.6f * Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
                                                           18.4f * Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)));
    light->Kd = Vector3f(0.65f);
	Material* MicrofacetMat=new Material(MaterialType::MicroFacet,Vector3f(0.0f));
	MicrofacetMat->Kd=Vector3f(0.2f,0.2f,0.2f);
	MicrofacetMat->Ks=Vector3f(0.7f,0.7f,0.7f);
	Sphere sphere(Vector3f(150.f,100.f,300.f),100.0,MicrofacetMat);

    MeshTriangle floor("../models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", white);
    MeshTriangle left("../models/cornellbox/left.obj", red);
    MeshTriangle right("../models/cornellbox/right.obj", green);
    MeshTriangle light_("../models/cornellbox/light.obj", light);

    scene.Add(&floor);
    scene.Add(&shortbox);
    scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    scene.Add(&sphere);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::high_resolution_clock::now();
    r.Render(scene);
    auto stop = std::chrono::high_resolution_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time token: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}