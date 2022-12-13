#include <nori/texturemapping.h>

NORI_NAMESPACE_BEGIN

/**
 * @brief Very simple UV mapping for textures
 */
class UVMapping2D : public TextureMapping2D {
    UVMapping2D(float su = 1.f, float sv = 1.f, float du = 0.0f, float dv = 0.0f) :
        m_su(su), m_sv(sv), m_du(du), m_dv(dv) {}

    UVMapping2D(const PropertyList &props) {
        m_su = props.getFloat("su", 1.0f);
        m_sv = props.getFloat("sv", 1.0f);
        m_du = props.getFloat("du", 0.0f);
        m_dv = props.getFloat("dv", 0.0f);
    }

    virtual Point2f Map(const Point2f &uv) const override {
        return Point2f(m_su * uv[0] + m_du, m_sv * uv[1] + m_dv);
    }
private:
    float m_su, m_sv, m_du, m_dv;
};

NORI_NAMESPACE_END