/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>
#include <nori/common.h>
#include <nori/transform.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Perspective camera with depth of field
 *
 * This class implements a simple perspective camera model. It uses an
 * infinitesimally small aperture, creating an infinite depth of field.
 */
class AdvancedCamera : public Camera {
public:
    AdvancedCamera(const PropertyList &propList) : 
        simpleWeighting(propList.getBoolean("simpleWeighting", true)) {

        /* Width and height in pixels. Default: 720p */
        m_outputSize.x() = propList.getInteger("width", 1280);
        m_outputSize.y() = propList.getInteger("height", 720);

        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());

        m_rfilter = NULL;
    }
    /**
     * \brief Perform some action associated with the object
     *
     * The default implementation throws an exception. Certain objects
     * may choose to override it, e.g. to implement initialization, 
     * testing, or rendering functionality.
     *
     * This function is called by the XML parser once it has
     * constructed an object and added all of its children
     * using \ref addChild().
     */
    virtual void activate() override {
        float aspect = m_outputSize.x() / (float) m_outputSize.y();

        /* If no reconstruction filter was assigned, instantiate a Gaussian filter */
        if (!m_rfilter) {
            m_rfilter = static_cast<ReconstructionFilter *>(
                    NoriObjectFactory::createInstance("gaussian", PropertyList()));
            m_rfilter->activate();
        }
    }

    /**
     * \brief Importance sample a ray according to the camera's response function
     *
     * \param ray
     *    A ray data structure to be filled with a position 
     *    and direction value
     *
     * \param samplePosition (pFilm)
     *    Denotes the desired sample position on the film
     *    expressed in fractional pixel coordinates
     *
     * \param apertureSample (pLens)
     *    A uniformly distributed 2D vector that is used to sample
     *    a position on the aperture of the sensor if necessary.
     *
     * \return
     *    An importance weight associated with the sampled ray.
     *    This accounts for the difference in the camera response
     *    function and the sampling density.
     */
    Color3f sampleRay(Ray3f &ray,
            const Point2f &samplePosition,
            const Point2f &apertureSample) const {

        return Color3f(1.0f);
    }

    virtual void addChild(NoriObject *obj) override {
        switch (obj->getClassType()) {
            case EReconstructionFilter:
                if (m_rfilter)
                    throw NoriException("Camera: tried to register multiple reconstruction filters!");
                m_rfilter = static_cast<ReconstructionFilter *>(obj);
                break;

            default:
                throw NoriException("Camera::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    /// Return a human-readable summary
    virtual std::string toString() const override {
        return tfm::format(
            "AdvancedCamera[\n"
            "  Element Interfaces = %s,\n"
            "  Exit Pupil Bounds = %s,\n"
            "]",
            indent(std::string(m_elementInterfaces.begin(), m_elementInterfaces.end()), 18),
            std::string(m_exitPupilBounds.begin(), m_exitPupilBounds.end())
        );
    }
private:
    // Define a lens element structure (based on PBRT)
    struct LensElementInterface {
        float curvatureRadius;
        float thickness;
        float eta;
        float apertureRadius;
    };

    //====================== ADVANCED CAMERA FIELDS ===================
    const bool simpleWeighting;
    std::vector<LensElementInterface> m_elementInterfaces;
    std::vector<BoundingBox2f> m_exitPupilBounds;
    Transform m_cameraToWorld;
    //=================================================================

    //=============== ADVANCED CAMERA PRIVATE METHODS =================
    // Z-coordinate of the rear lens in our system
    float LensRearZ() const { 
        return m_elementInterfaces.back().thickness; 
    }

    // Z-coordinate of the front lens in our system
    float LensFrontZ() const {
        float zSum = 0;
        for (const LensElementInterface &element : m_elementInterfaces)
            zSum += element.thickness;
        return zSum;
    }

    // The aperture radius of the rear lens in our system
    float RearElementRadius() const {
        return m_elementInterfaces.back().apertureRadius;
    }

    // Given a ray starting from the sensor, compute the intersection
    // with each element one by one.
    bool TraceLensesFromFilm(const Ray3f &ray, Ray3f *rOut) const {
        float elementZ = 0.0f;
        Matrix4f tf(1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, -1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f);
        // Transform the ray form the camera space to the lens space
        static const Transform cameraToLens = Transform(tf, tf);
        Ray3f rayL = cameraToLens(ray);

        // Pass ray through each lens in the system
        for(int i = m_elementInterfaces.size() - 1; i >= 0; --i) {
            const LensElementInterface &elem = m_elementInterfaces[i];

            // Account for the interaction with the lens element
            elementZ -= elem.thickness;

            // Compute intersection of ray with element
            float t;
            Normal3f n;
            bool isStop = (elem.curvatureRadius == 0);

            if(isStop) {
                t = (elementZ - rayL.o.z()) / rayL.d.z();
            } else {
                float radius = elem.curvatureRadius;
                float zC = elementZ + elem.curvatureRadius;

                // Check for an intersection with the element
                if(!IntersectSphericalElement(radius, zC, rayL, &t, &n)) {
                    return false;
                }
            }

            // Test the intersection point against the element aperture
            Point3f pHit = rayL(t);
            float r2 = pHit.x() * pHit.x() + pHit.y() * pHit.y();
            if(r2 > elem.apertureRadius * elem.apertureRadius) {
                return false;
            }
            rayL.o = pHit;

            // Update the ray path to take into account the element interaction
            if(!isStop) {
                Vector3f w;
                float etaI = elem.eta;
                float etaT = (i > 0 && elementInterfaces[i - 1].eta != 0) ?
                    elementInterfaces[i - 1].eta : 1.0f;

                if(!refract((-rayL.d).normalized(), n, etaI / etaT, &w)) {
                    return false;
                }
                rayL.d = w;
            }
        }
        // Transform rayL from the lens space back into the camera space
        if(rOut) {
            Matrix4f tf(1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, -1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f);
            // Transform the ray form the camera space to the lens space
            static const Transform lensToCamera = Transform(tf, tf);
            *rOut = lensToCamera(rayL);
        }
        return true;
    }

    // Computes the Intersection with a spherical element
    static bool IntersectSphericalElement(float radius, float zCenter, const Ray3f &ray, float *t, Normal3f *n) {
        // Compute the two t values for both entry and exit intersection points
        Point3f o = ray.o - Vector3f(0.0f, 0.0f, zCenter);
        // Compute the 3 variables of our implicit equation
        float A = ray.d.x() * ray.d.x() + ray.d.y() * ray.d.y() + ray.d.z() * ray.d.z(); 
        float B = 2.0f * (ray.d.x() * o.x() + ray.d.y() * o.y() + ray.d.z() * o.z());
        float C = o.x() * o.x() + o.y() * o.y() + o.z() * o.z() - radius * radius;
        
        // Solve the implicit equation
        float t0, t1;
        size_t nSol = solve_quadratic(A, B, C, &t0, &t1);
        if(nSol == 0) {
            return false; // If no solution was found then there is no intersection
        }
        
        // Select which intersection t we must use
        // based on the ray dir and elem curvature
        bool useCloserT = (ray.d.z() > 0) ^ (radius < 0);
        // Check that more than one solution exists
        if(nSol == 1) {
            *t = t0;
        } else {
            *t = useCloserT ? std::min(t0, t1) : std::max(t0, t1);
        }
        if(*t < 0) { // is intersection behind the camera
            return false;
        }

        // Compute the surface normal of the element at the intersection point
        Vector3f its = Vector3f(o + *t * ray.d);
        *n = Normal3f(its);
        *n = Faceforward((*n).normalized(), -ray.d);

        // At this point we are sure that an intersection has been found
        return true;
    }

    // Same idea as LensesFromFilm but the other way through the lens system
    bool TraceLensesFromScene(const Ray3f &ray, Ray3f *rOut) const {
        float elementZ = -LensFrontZ();
        Matrix4f tf(1.0f, 0.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, -1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f);
        // Transform the ray form the camera space to the lens space
        static const Transform cameraToLens = Transform(tf, tf);
        Ray3f rayL = cameraToLens(ray);

        // Pass ray through each lens in the system
        for(int i = 0; i < m_elementInterfaces.size(); ++i) {
            const LensElementInterface &elem = m_elementInterfaces[i];

            // Compute intersection of ray with element
            float t;
            Normal3f n;
            bool isStop = (elem.curvatureRadius == 0);

            if(isStop) {
                t = (elementZ - rayL.o.z()) / rayL.d.z();
            } else {
                float radius = elem.curvatureRadius;
                float zC = elementZ + elem.curvatureRadius;

                // Check for an intersection with the element
                if(!IntersectSphericalElement(radius, zC, rayL, &t, &n)) {
                    return false;
                }
            }

            // Test the intersection point against the element aperture
            Point3f pHit = rayL(t);
            float r2 = pHit.x() * pHit.x() + pHit.y() * pHit.y();
            if(r2 > elem.apertureRadius * elem.apertureRadius) {
                return false;
            }
            rayL.o = pHit;

            // Update the ray path to take into account the element interaction
            if(!isStop) {
                Vector3f w;
                float etaI = (i == 0 || elementInterfaces[i - 1].eta == 0) ? 
                            1 : elementInterfaces[i - 1].eta;
                float etaT = (elementInterfaces[i].eta != 0) ?
                             elementInterfaces[i].eta : 1;

                if(!refract((-rayL.d).normalized(), n, etaI / etaT, &w)) {
                    return false;
                }
                rayL.d = w;
            }
            elementZ += elem.thickness;
        }

        // Transform rayL from the lens space back into the camera space
        if(rOut) {
            Matrix4f tf(1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, -1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f);
            // Transform the ray form the camera space to the lens space
            static const Transform lensToCamera = Transform(tf, tf);
            *rOut = lensToCamera(rayL);
        }
        return true;
    }

    void DrawLensSystem() const;
    void DrawRayPathFromFilm(const Ray3f &r, bool arrow,
                            bool toOpticalIntercept) const;
    void DrawRayPathFromScene(const Ray3f &r, bool arrow,
                            bool toOpticalIntercept) const;
    static void ComputeCardinalPoints(const Ray3f &rIn, const Ray3f &rOut, float *p,
                                    float *f);
    void ComputeThickLensApproximation(float pz[2], float f[2]) const;
    float FocusThickLens(float focusDistance);
    float FocusBinarySearch(float focusDistance);
    float FocusDistance(float filmDist);
    BoundingBox2f BoundExitPupil(float pFilmX0, float pFilmX1) const;
    void RenderExitPupil(float sx, float sy, const char *filename) const;
    Point3f SampleExitPupil(const Point2f &pFilm, const Point2f &lensSample,
                            float *sampleBoundsArea) const;
    void TestExitPupilBounds() const;
    //========================================================================

};

NORI_REGISTER_CLASS(AdvancedCamera, "advancedCamera");
NORI_NAMESPACE_END
