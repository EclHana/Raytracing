#include "Triangle.hpp"
#include "OBJ_Loader.hpp"

bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1,
                          const Vector3f &v2, const Vector3f &orig,
                          const Vector3f &dir, float &tnear, float &u, float &v)
{
    /*
        三角形内一点可表示为
            P = (1-u-v)A + uB + vC
            P = A + u(B-A) + v(C-A)

        光线运行中的点可表示为
            Pr = O + t*Dir

        三角形与光线相交的点:
            A + u(B-A) + v(C-A) = O + t*Dir

            O-A = -t*Dir + u(B-A) + v(C-A)

        对方程中的x,y,z维度列方程组解出t,u,v

    */

    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    Vector3f pvec = crossProduct(dir, edge2);
    float det = dotProduct(edge1, pvec);
    if (det <= 0)
        return false;

    Vector3f tvec = orig - v0;
    u = dotProduct(tvec, pvec);
    if (u < 0 || u > det)
        return false;

    Vector3f qvec = crossProduct(tvec, edge1);
    v = dotProduct(dir, qvec);
    if (v < 0 || u + v > det)
        return false;

    float invDet = 1 / det;

    tnear = dotProduct(edge2, qvec) * invDet;

    u *= invDet;
    v *= invDet;

    return true;
}

bool rayTriangleIntersect(const Triangle &tr, const Vector3f &orig,
                          const Vector3f &dir, float &tnear, float &u, float &v)
{
    return rayTriangleIntersect(tr.v0, tr.v1, tr.v2, orig, dir, tnear, u, v);
}

Triangle::Triangle(Vector3f &_v0, Vector3f &_v1, Vector3f &_v2, Material *_m)
    : v0(_v0), v1(_v1), v2(_v2), m(_m)
{
    e1 = v1 - v0;
    e2 = v2 - v0;
    normal = normalize(crossProduct(e1, e2));
    area =  crossProduct(e1, e2).norm()*0.5f; // 叉乘的模等于两个edge围成的平行四边形的面积
}

bool Triangle::intersect(const Ray &ray)
{
	return true;
}
bool Triangle::intersect(const Ray &ray, float &tnear, uint32_t &index) const
{
	return false;
}

Intersection Triangle::getIntersection(const Ray &ray)
{
    if (dotProduct(ray.direction, normal) > 0)
        return Intersection();

    double u, v, t_tmp = 0;
    Vector3f pvec = crossProduct(ray.direction, e2);
    double det = dotProduct(e1, pvec);
    if (fabs(det) < EPSILON)
        return Intersection();

    double InvDet = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = dotProduct(tvec, pvec) * InvDet;
    if (u < 0 || u > 1)
        return Intersection();

    Vector3f qvec = crossProduct(tvec, e1);
    v = dotProduct(ray.direction, qvec) * InvDet;
    if (v < 0 || v + u > 1)
        return Intersection();

    t_tmp = dotProduct(e2, qvec) * InvDet;

    if(t_tmp<0) return Intersection();
    Intersection inter;
    inter.happened = true;
    inter.coords = ray(t_tmp);
    inter.normal = normal;
    inter.distance = t_tmp;
    inter.m = m;
    inter.obj = this;
    return inter;
}

Vector3f Triangle::evalDiffuseColor(const Vector2f &) const
{
    return Vector3f(0.5, 0.5, 0.5);
}

Bounds3 Triangle::getBounds()
{
    return Bounds3::Union(Bounds3(v0, v1), v2);
}

void Triangle::Sample(Intersection &pos, float &pdf)
{
	float x = std::sqrt(get_random_float()), y = get_random_float();
	pos.coords = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
	pos.normal = this->normal;
	pdf = 1.0f / area; // area上均匀分布
}

// MeshTriangle

MeshTriangle::MeshTriangle(const std::string filename, Material *mt)
{
	objl::Loader loader;
	loader.LoadFile(filename);
	area = 0;
	m = mt;
	auto mesh = loader.LoadedMeshes[0];

	Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
	                             std::numeric_limits<float>::infinity(),
	                             std::numeric_limits<float>::infinity()};
	Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
	                             -std::numeric_limits<float>::infinity(),
	                             -std::numeric_limits<float>::infinity()};
	for (int i = 0; i < mesh.Vertices.size(); i += 3) {
		std::array<Vector3f, 3> face_vertices;

		for (int j = 0; j < 3; j++) {
			auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
			                     mesh.Vertices[i + j].Position.Y,
			                     mesh.Vertices[i + j].Position.Z);
			face_vertices[j] = vert;

			min_vert = Vector3f(std::min(min_vert.x, vert.x),
			                    std::min(min_vert.y, vert.y),
			                    std::min(min_vert.z, vert.z));
			max_vert = Vector3f(std::max(max_vert.x, vert.x),
			                    std::max(max_vert.y, vert.y),
			                    std::max(max_vert.z, vert.z));
		}

		triangles.emplace_back(face_vertices[0], face_vertices[1],
		                       face_vertices[2], mt);
	}

	bounding_box = Bounds3(min_vert, max_vert);

	std::vector<Object*> ptrs;
	for (auto& tri : triangles){
		ptrs.push_back(&tri);
		area += tri.area;
	}
	bvh = new BVHAccel(ptrs);
}

bool MeshTriangle::intersect(const Ray &ray)
{
	return true;
}

bool MeshTriangle::intersect(const Ray &ray, float &tnear, uint32_t &index) const
{
    bool res = false;
    for (uint32_t k = 0; k < numTriangles; k++)
    {
        float t, u, v;
        if (rayTriangleIntersect(triangles[k], ray.origin, ray.direction, t, u, v) && t < tnear)
        {
            tnear = t;
            index = k;
            res |= true;
        }
    }
    return res;
}

void MeshTriangle::getSurfaceProperties(const Vector3f &P, const Vector3f &I,
                                        const uint32_t &index, const Vector2f &uv,
                                        Vector3f &N, Vector2f &st) const
{
    const Vector3f &v0 = vertices[vertexIndex[index * 3]];
    const Vector3f &v1 = vertices[vertexIndex[index * 3 + 1]];
    const Vector3f &v2 = vertices[vertexIndex[index * 3 + 2]];
    Vector3f e0 = normalize(v1 - v0);
    Vector3f e1 = normalize(v2 - v1);
    N = normalize(crossProduct(e0, e1));
    const Vector2f &st0 = stCoordinates[vertexIndex[index * 3]];
    const Vector2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]];
    const Vector2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]];
    st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
}

Vector3f MeshTriangle::evalDiffuseColor(const Vector2f &st) const
{
    float scale = 5;
    float pattern =
        (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
    return lerp(Vector3f(0.815, 0.235, 0.031),
                Vector3f(0.937, 0.937, 0.231), pattern);
}

Intersection MeshTriangle::getIntersection(const Ray &ray)
{
    return bvh ? bvh->Intersect(ray) : Intersection();
}

void MeshTriangle::Sample(Intersection &pos, float &pdf)
{
    bvh->Sample(pos, pdf);
    pos.emit = m->getEmission();
}