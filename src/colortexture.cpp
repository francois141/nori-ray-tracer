#include <nori/texture.h>
#include <nori/common.h>
#include <nori/color.h>

NORI_NAMESPACE_BEGIN

class ColorTexture : public Texture<Color3f> {
    public:

    ColorTexture(const PropertyList &props) {
        m_value = props.getColor("value", Color3f(1.0f));
    }

    // Constant textures just return their value
    virtual Color3f eval(const Point2f & uv) {
        return m_value;
    }

private:
    Color3f m_value;
};
NORI_NAMESPACE_END
