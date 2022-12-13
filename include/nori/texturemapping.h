#if !defined(__NORI_TEXTUREMAPPING_H)
#define __NORI_TEXTUREMAPPING_H

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

class TextureMapping2D {
public:
    virtual ~TextureMapping2D() {}

    virtual Point2f Map(const Point2f &uv) const = 0;
};

NORI_NAMESPACE_END

#endif
