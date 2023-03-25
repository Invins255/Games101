//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    
    
    //若光线能够与空间中的物体发生碰撞
    if (inter.happened) {

        //若该物体为光源，直接返回光源光照
        if (inter.m->hasEmission()) {
            if (depth == 0)
                return inter.m->getEmission();
            else
                return Vector3f(0, 0, 0);
        }

        Vector3f L_dir = Vector3f(0, 0, 0);         //直接光照
        Vector3f L_indir = Vector3f(0, 0, 0);       //间接光照

        //对光源继续采样，检测着色点到光源采样点的射线是否与其他物体碰撞
        Intersection lightInter;
        float pdf_light = 0.0f;
        sampleLight(lightInter, pdf_light);

        auto& objectN = inter.normal;               //着色点法线
        auto& lightN = lightInter.normal;           //光源采样点法线
        auto& objectPos = inter.coords;             //着色点坐标
        auto& lightPos = lightInter.coords;         //光源采样点坐标

        auto diff = lightPos - objectPos;
        auto lightDir = diff.normalized();
        auto lightDistance = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;

        Ray light = Ray(objectPos, lightDir);
        Intersection objInter = intersect(light);

        //射线能够直接碰撞到光源上，计算直接光照
        if (objInter.happened && (objInter.coords - lightPos).norm() < 1e-2) {
            Vector3f f_r = inter.m->eval(ray.direction, lightDir, objectN);
            L_dir = lightInter.emit * f_r * dotProduct(lightDir, objectN) * dotProduct(-lightDir, lightN) / lightDistance / pdf_light;
        }

        //计算间接光照(使用RR判断光线是否停止弹射)
        if (get_random_float() < RussianRoulette) {
            //选取一个方向进行采样
            Vector3f nextDir = inter.m->sample(ray.direction, objectN).normalized();

            Ray nextRay = Ray(objectPos, nextDir);
            Intersection nextInter = intersect(nextRay);
            if (nextInter.happened && !nextInter.m->hasEmission()) {
                float pdf = inter.m->pdf(ray.direction, nextDir, objectN);
                Vector3f f_r = inter.m->eval(ray.direction, nextDir, objectN);
                L_indir = castRay(nextRay, depth + 1) * f_r * dotProduct(nextDir, objectN) / pdf / RussianRoulette;
            }
        }

        return L_dir + L_indir;
    }

    
    return Vector3f(0, 0, 0);
}
