#ifndef MX_XD_UTILS_H
#define MX_XD_UTILS_H

#include "rbdl/CasadiMath/MX_Xd_scalar.h"
#include "rbdl/CasadiMath/MX_Xd_static.h"
#include "rbdl/CasadiMath/MX_Xd_dynamic.h"
#include "rbdl/CasadiMath/MX_Xd_subMatrix.h"


namespace RBDLCasadiMath {

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator*(
        const MX_Xd_static<nrows, ncols>& me,
        const MX_Xd_scalar& other) {
    return casadi::MX::mtimes(me, other);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator*(
        const MX_Xd_static<nrows, ncols>& me,
        const double& other) {
    return casadi::MX::mtimes(me, other);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator*(
        const MX_Xd_scalar& other,
        const MX_Xd_static<nrows, ncols>& me
        ) {
    return casadi::MX::mtimes(me, other);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator*(
        const double& other,
        const MX_Xd_static<nrows, ncols>& me
        ) {
    return casadi::MX::mtimes(me, other);
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator+(
        const MX_Xd_static<nrows, ncols>& me,
        const MX_Xd_dynamic& other) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator+=(other);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator+(
        const MX_Xd_static<nrows, ncols>& me,
        const MX_Xd_SubMatrix& other) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator+=(other);
}
inline MX_Xd_scalar operator+(
        MX_Xd_scalar me,
        const MX_Xd_SubMatrix& other) {
    return me.casadi::MX::operator+=(other);
}
inline MX_Xd_scalar operator+(
        MX_Xd_scalar me,
        const double& other) {
    return me.casadi::MX::operator+=(other);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator+(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_static<nrows, ncols>& other) {
    MX_Xd_static<nrows, ncols> out(other);
    return out.casadi::MX::operator+=(me);
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator-(
        const MX_Xd_static<nrows, ncols>& other) {
    return other.casadi::MX::operator-();
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator-(
        const MX_Xd_static<nrows, ncols>& me,
        const MX_Xd_dynamic& other) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator-=(other);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator-(
        const MX_Xd_static<nrows, ncols>& me,
        const MX_Xd_SubMatrix& other) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator-=(other);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator-(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_static<nrows, ncols>& other) {
    MX_Xd_static<nrows, ncols> out(other);
    return out.casadi::MX::operator-=(me);
}

inline MX_Xd_dynamic operator*(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_scalar& other){
    return MX_Xd_SubMatrix::times(me, other);
}
inline MX_Xd_dynamic operator*(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_SubMatrix& other){
    return MX_Xd_SubMatrix::mtimes(me, other);
}

inline MX_Xd_SubMatrix operator/(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_SubMatrix& scalar) {
    MX_Xd_SubMatrix result (me);
    result.MX_Xd_SubMatrix::operator/=(scalar);
    return result;
}
inline MX_Xd_SubMatrix operator/(
        const MX_Xd_SubMatrix& me,
        const double& scalar) {
    MX_Xd_SubMatrix result (me);
    result.MX_Xd_SubMatrix::operator/=(scalar);
    return result;
}

inline MX_Xd_dynamic operator*(
        const MX_Xd_scalar& m1,
        const MX_Xd_dynamic m2){
    return casadi::MX::mtimes(m1, m2);
}
inline MX_Xd_dynamic operator*(
        const double& m1,
        const MX_Xd_dynamic m2){
    return casadi::MX::mtimes(m1, m2);
}
inline MX_Xd_dynamic operator*(
        const double& m1,
        const MX_Xd_SubMatrix m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_static<nrows, ncols>& m1,
        const MX_Xd_dynamic m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_dynamic m1,
        const MX_Xd_static<nrows, ncols>& m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_SubMatrix& m1,
        const MX_Xd_static<nrows, ncols>& m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_static<nrows, ncols>& m1,
        const MX_Xd_SubMatrix& m2){
    return casadi::MX::mtimes(m1, m2);
}

inline MX_Xd_dynamic operator/(double scalar, MX_Xd_SubMatrix mat){
    for (unsigned int i=0; i<mat.rows(); ++i){
        for (unsigned int j=0; j<mat.columns(); ++j){
            mat(i, j) = scalar / mat(i, j);
        }
    }
    return mat;
}
inline MX_Xd_dynamic operator/(double scalar, MX_Xd_dynamic mat){
    for (unsigned int i=0; i<mat.rows(); ++i){
        for (unsigned int j=0; j<mat.columns(); ++j){
            mat(i, j) = scalar / mat(i, j);
        }
    }
    return mat;
}
inline MX_Xd_scalar operator/(double scalar, const MX_Xd_scalar& mat){
    MX_Xd_scalar out(scalar);
    return out.casadi::MX::operator/=(mat(0, 0));
}
template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_dynamic operator/(double scalar, MX_Xd_static<nrows, ncols> mat){
    for (unsigned int i=0; i<nrows; ++i){
        for (unsigned int j=0; j<ncols; ++j){
            mat(i, j) = scalar / mat(i, j);
        }
    }
    return mat;
}


template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator+(
        const MX_Xd_dynamic& me,
        const MX_Xd_static<nrows, ncols>& other
        ) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator+=(other);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_dynamic& me) {
    return me.casadi::MX::operator-();
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_SubMatrix& me) {
    return me.casadi::MX::operator-();
}
inline MX_Xd_dynamic operator-(
        MX_Xd_SubMatrix me,
        const MX_Xd_scalar& scalar){
    return me.casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator-=(scalar);
}
inline MX_Xd_scalar operator-(
        MX_Xd_scalar me,
        const MX_Xd_SubMatrix& other) {
    return me.casadi::MX::operator-=(other);
}
inline MX_Xd_scalar operator+(
        const MX_Xd_dynamic& me,
        const MX_Xd_scalar& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator+=(other);
}
inline MX_Xd_scalar operator+(
        const MX_Xd_scalar& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator+=(other);
}
inline MX_Xd_dynamic operator+(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_dynamic out(other);
    return out.casadi::MX::operator+=(me);
}
inline MX_Xd_dynamic operator+(
        const MX_Xd_dynamic& me,
        const MX_Xd_SubMatrix& other
        ) {
    MX_Xd_dynamic out(me);
    return out.casadi::MX::operator+=(other);
}

inline MX_Xd_dynamic operator+(
        MX_Xd_SubMatrix me,
        const MX_Xd_SubMatrix& other){
    return me.casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator+=(other);
}

template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_static<nrows, ncols> operator-(
        const MX_Xd_dynamic& me,
        const MX_Xd_static<nrows, ncols>& other
        ) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_scalar operator-(
        const MX_Xd_dynamic& me,
        const MX_Xd_scalar& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_scalar operator-(
        const MX_Xd_scalar& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_dynamic out(-other);
    return out.casadi::MX::operator+=(me);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_dynamic& me,
        const MX_Xd_SubMatrix& other
        ) {
    MX_Xd_dynamic out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_SubMatrix& other
        ) {
    MX_Xd_dynamic out(me);
    return out.casadi::MX::operator-=(other);
}


inline MX_Xd_scalar operator*(
        double other,
        const MX_Xd_scalar& me
        ) {
    return casadi::MX::times(me, other);
}
inline MX_Xd_scalar operator*(
        const MX_Xd_scalar& other,
        const MX_Xd_SubMatrix& me
        ) {
    return casadi::MX::times(me, other);
}

inline MX_Xd_scalar operator+(
        double other,
        const MX_Xd_scalar& me) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator+=(other);
}

inline MX_Xd_scalar operator-(
        const MX_Xd_scalar& other) {
    return other.casadi::MX::operator-();
}
inline MX_Xd_scalar operator-(
        double other,
        const MX_Xd_scalar& me) {
    MX_Xd_scalar out(-me);
    return out.casadi::MX::operator+=(other);
}

}


inline RBDLCasadiMath::MX_Xd_scalar exp(const RBDLCasadiMath::MX_Xd_scalar& x){
    return casadi::MX::exp(x);
}

namespace std {
using namespace RBDLCasadiMath;

inline MX_Xd_scalar sqrt(const MX_Xd_scalar& x){
    return casadi::MX::sqrt(x);
}
inline MX_Xd_scalar log(const MX_Xd_scalar& x){
    return casadi::MX::log(x);
}
inline MX_Xd_scalar sin(const MX_Xd_scalar& x){
    return casadi::MX::sin(x);
}
inline MX_Xd_scalar asin(const MX_Xd_scalar& x){
    return casadi::MX::asin(x);
}
inline MX_Xd_scalar cos(const MX_Xd_scalar& x){
    return casadi::MX::cos(x);
}
inline MX_Xd_scalar acos(const MX_Xd_scalar& x){
    return casadi::MX::acos(x);
}
inline MX_Xd_scalar tan(const MX_Xd_scalar& x){
    return casadi::MX::tan(x);
}
inline MX_Xd_scalar atan2(const MX_Xd_scalar& x, const MX_Xd_scalar& y){
    return casadi::MX::atan2(x,y);
}
inline MX_Xd_scalar tanh(const MX_Xd_scalar& x){
    return casadi::MX::tanh(x);
}
inline bool isnan(const casadi::MX& x){
    return !x.is_regular();
}

template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_dynamic fabs(const MX_Xd_dynamic& m){
    return casadi::MX::abs(m);
}
template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_scalar fabs(const MX_Xd_static<nrows, ncols>& m){
    return casadi::MX::abs(m);
}
inline MX_Xd_scalar fabs(const MX_Xd_scalar& m){
    return casadi::MX::abs(m);
}

template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_static<nrows, ncols> pow(const MX_Xd_static<nrows, ncols>& m, int exponent){
    return casadi::MX::mpower(m, exponent);
}
template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_static<nrows, ncols> pow(const MX_Xd_static<nrows, ncols>& m, unsigned int exponent){
    return casadi::MX::mpower(m, exponent);
}
template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_static<nrows, ncols> pow(const MX_Xd_static<nrows, ncols>& m, double exponent){
    return casadi::MX::mpower(m, exponent);
}
inline MX_Xd_scalar pow(const MX_Xd_scalar& m, int exponent){
    return casadi::MX::mpower(m, exponent);
}
inline MX_Xd_scalar pow(const MX_Xd_scalar& m, unsigned int exponent){
    return casadi::MX::mpower(m, exponent);
}
inline MX_Xd_scalar pow(const MX_Xd_scalar& m, double exponent){
    return casadi::MX::mpower(m, exponent);
}

}

#endif
