#include "visionmodels.hpp"

template <class T>
void rgb2rg_chromasity(T r, T g, T b, T& rc, T& gc) {
    T intensity = r+b+g;
    rc = r/intensity;
    gc = g/intensity;
}
