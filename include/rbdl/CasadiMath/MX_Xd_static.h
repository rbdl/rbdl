/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef MX_XD_STATIC_H
#define MX_XD_STATIC_H

#include <vector>
#include <string>
#include <memory>

#include <casadi.hpp>
#include "MX_Xd_scalar.h"
#include "MX_Xd_dynamic.h"


namespace RBDLCasadiMath {

template <unsigned int nrows, unsigned int ncols>
class MX_Xd_static : public casadi::MX{
public:
    MX_Xd_static() : casadi::MX(nrows, ncols){

    }

    virtual ~MX_Xd_static(){

    }

    MX_Xd_static(const double val) : casadi::MX(1, 1)
    {
        (*this)(0, 0) = val;
    }

    MX_Xd_static(const casadi::MX& m) : casadi::MX(m){
    }

    MX_Xd_static(
            const MX_Xd_scalar& v0,
            const MX_Xd_scalar& v1) :
        casadi::MX(2, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
    }
    MX_Xd_static(
            const MX_Xd_scalar& v0,
            const MX_Xd_scalar& v1,
            const MX_Xd_scalar& v2) :
        casadi::MX(3, 1)
    {
        this->casadi::MX::operator ()(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);

    }
    MX_Xd_static(const MX_Xd_scalar& v0,
                 const MX_Xd_scalar& v1,
                 const MX_Xd_scalar& v2,
                 const MX_Xd_scalar& v3) :
        casadi::MX(4, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
        (*this)(3) = v3(0);
    }
    MX_Xd_static(const MX_Xd_scalar& v0,
                 const MX_Xd_scalar& v1,
                 const MX_Xd_scalar& v2,
                 const MX_Xd_scalar& v3,
                 const MX_Xd_scalar& v4,
                 const MX_Xd_scalar& v5) :
        casadi::MX(6, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
        (*this)(3) = v3(0);
        (*this)(4) = v4(0);
        (*this)(5) = v5(0);
    }

    MX_Xd_static(const MX_Xd_scalar& v0,
                 const MX_Xd_scalar& v1,
                 const MX_Xd_scalar& v2,
                 const MX_Xd_scalar& v3,
                 const MX_Xd_scalar& v4,
                 const MX_Xd_scalar& v5,
                 const MX_Xd_scalar& v6,
                 const MX_Xd_scalar& v7) :
        casadi::MX(8, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
        (*this)(3) = v3(0);
        (*this)(4) = v4(0);
        (*this)(5) = v5(0);
        (*this)(6) = v6(0);
        (*this)(7) = v7(0);

    }
    MX_Xd_static(const MX_Xd_scalar& v00, const MX_Xd_scalar& v01, const MX_Xd_scalar& v02,
                 const MX_Xd_scalar& v10, const MX_Xd_scalar& v11, const MX_Xd_scalar& v12,
                 const MX_Xd_scalar& v20, const MX_Xd_scalar& v21, const MX_Xd_scalar& v22) :
        casadi::MX(3, 3)
    {
        (*this)(0,0) = v00(0, 0);
        (*this)(0,1) = v01(0, 0);
        (*this)(0,2) = v02(0, 0);
        (*this)(1,0) = v10(0, 0);
        (*this)(1,1) = v11(0, 0);
        (*this)(1,2) = v12(0, 0);
        (*this)(2,0) = v20(0, 0);
        (*this)(2,1) = v21(0, 0);
        (*this)(2,2) = v22(0, 0);
    }

    MX_Xd_static(const MX_Xd_scalar& v00, const MX_Xd_scalar& v01, const MX_Xd_scalar& v02, const MX_Xd_scalar& v03,
                 const MX_Xd_scalar& v10, const MX_Xd_scalar& v11, const MX_Xd_scalar& v12, const MX_Xd_scalar& v13,
                 const MX_Xd_scalar& v20, const MX_Xd_scalar& v21, const MX_Xd_scalar& v22, const MX_Xd_scalar& v23,
                 const MX_Xd_scalar& v30, const MX_Xd_scalar& v31, const MX_Xd_scalar& v32, const MX_Xd_scalar& v33) :
        casadi::MX(4, 4)
    {
        (*this)(0,0) = v00(0, 0);
        (*this)(0,1) = v01(0, 0);
        (*this)(0,2) = v02(0, 0);
        (*this)(0,3) = v03(0, 0);
        (*this)(1,0) = v10(0, 0);
        (*this)(1,1) = v11(0, 0);
        (*this)(1,2) = v12(0, 0);
        (*this)(1,3) = v13(0, 0);
        (*this)(2,0) = v20(0, 0);
        (*this)(2,1) = v21(0, 0);
        (*this)(2,2) = v22(0, 0);
        (*this)(2,3) = v23(0, 0);
        (*this)(3,0) = v30(0, 0);
        (*this)(3,1) = v31(0, 0);
        (*this)(3,2) = v32(0, 0);
        (*this)(3,3) = v33(0, 0);
    }

    MX_Xd_static(const MX_Xd_scalar& v00, const MX_Xd_scalar& v01, const MX_Xd_scalar& v02, const MX_Xd_scalar& v03, const MX_Xd_scalar& v04, const MX_Xd_scalar& v05,
                 const MX_Xd_scalar& v10, const MX_Xd_scalar& v11, const MX_Xd_scalar& v12, const MX_Xd_scalar& v13, const MX_Xd_scalar& v14, const MX_Xd_scalar& v15,
                 const MX_Xd_scalar& v20, const MX_Xd_scalar& v21, const MX_Xd_scalar& v22, const MX_Xd_scalar& v23, const MX_Xd_scalar& v24, const MX_Xd_scalar& v25,
                 const MX_Xd_scalar& v30, const MX_Xd_scalar& v31, const MX_Xd_scalar& v32, const MX_Xd_scalar& v33, const MX_Xd_scalar& v34, const MX_Xd_scalar& v35,
                 const MX_Xd_scalar& v40, const MX_Xd_scalar& v41, const MX_Xd_scalar& v42, const MX_Xd_scalar& v43, const MX_Xd_scalar& v44, const MX_Xd_scalar& v45,
                 const MX_Xd_scalar& v50, const MX_Xd_scalar& v51, const MX_Xd_scalar& v52, const MX_Xd_scalar& v53, const MX_Xd_scalar& v54, const MX_Xd_scalar& v55) :
        casadi::MX(6, 6)
    {
        (*this)(0,0) = v00(0, 0);
        (*this)(0,1) = v01(0, 0);
        (*this)(0,2) = v02(0, 0);
        (*this)(0,3) = v03(0, 0);
        (*this)(0,4) = v04(0, 0);
        (*this)(0,5) = v05(0, 0);

        (*this)(1,0) = v10(0, 0);
        (*this)(1,1) = v11(0, 0);
        (*this)(1,2) = v12(0, 0);
        (*this)(1,3) = v13(0, 0);
        (*this)(1,4) = v14(0, 0);
        (*this)(1,5) = v15(0, 0);

        (*this)(2,0) = v20(0, 0);
        (*this)(2,1) = v21(0, 0);
        (*this)(2,2) = v22(0, 0);
        (*this)(2,3) = v23(0, 0);
        (*this)(2,4) = v24(0, 0);
        (*this)(2,5) = v25(0, 0);

        (*this)(3,0) = v30(0, 0);
        (*this)(3,1) = v31(0, 0);
        (*this)(3,2) = v32(0, 0);
        (*this)(3,3) = v33(0, 0);
        (*this)(3,4) = v34(0, 0);
        (*this)(3,5) = v35(0, 0);

        (*this)(4,0) = v40(0, 0);
        (*this)(4,1) = v41(0, 0);
        (*this)(4,2) = v42(0, 0);
        (*this)(4,3) = v43(0, 0);
        (*this)(4,4) = v44(0, 0);
        (*this)(4,5) = v45(0, 0);

        (*this)(5,0) = v50(0, 0);
        (*this)(5,1) = v51(0, 0);
        (*this)(5,2) = v52(0, 0);
        (*this)(5,3) = v53(0, 0);
        (*this)(5,4) = v54(0, 0);
        (*this)(5,5) = v55(0, 0);
    }

    ///
    /// \brief set For 3d Vector
    /// \param v0 X
    /// \param v1 Y
    /// \param v2 Z
    ///
    void set(
            const MX_Xd_scalar& v0,
            const MX_Xd_scalar& v1,
            const MX_Xd_scalar& v2){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
    }
    ///
    /// \brief set For Quaternion
    /// \param v0 X
    /// \param v1 Y
    /// \param v2 Z
    /// \param v3 W
    ///
    void set(
            const MX_Xd_scalar& v0,
            const MX_Xd_scalar& v1,
            const MX_Xd_scalar& v2,
            const MX_Xd_scalar& v3){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
        (*this)(3) = v3;
    }
    ///
    /// \brief set For SpatialVector
    /// \param v0
    /// \param v1
    /// \param v2
    /// \param v3
    /// \param v4
    /// \param v5
    ///
    void set(
            const MX_Xd_scalar& v0,
            const MX_Xd_scalar& v1,
            const MX_Xd_scalar& v2,
            const MX_Xd_scalar& v3,
            const MX_Xd_scalar& v4,
            const MX_Xd_scalar& v5){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
        (*this)(3) = v3;
        (*this)(4) = v4;
        (*this)(5) = v5;
    }

    static MX_Xd_static Identity(){
        return casadi::MX::eye(ncols);
    }
    MX_Xd_static& setIdentity(){
        *this = casadi::MX::eye(ncols);
        return *this;
    }

    static MX_Xd_static Zero(){
        return MX_Xd_static<nrows, ncols>::zeros(nrows, ncols);
    }
    MX_Xd_static& setZero(){
        *this = casadi::MX::zeros(this->rows(), this->cols());
        return *this;
    }

    static MX_Xd_static One(){
        return MX_Xd_static<nrows, ncols>::ones(nrows, ncols);
    }
    MX_Xd_static& setOnes(){
        *this = casadi::MX::ones(this->rows(), this->cols());
        return *this;
    }


    unsigned int rows() const {
        return static_cast<unsigned int>(this->casadi::MX::rows());
    }

    unsigned int cols() const {
        return static_cast<unsigned int>(this->casadi::MX::columns());
    }

    unsigned int size() const {
        return rows() * cols();
    }

    template <unsigned int row_count, unsigned int col_count>
    MX_Xd_SubMatrix block (
            unsigned int row_start,
            unsigned int col_start)
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }
    template <unsigned int row_count, unsigned int col_count>
    MX_Xd_static<row_count, col_count> block(
            unsigned int row_start,
            unsigned int col_start) const
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }
    MX_Xd_SubMatrix block(
            unsigned int row_start,
            unsigned int col_start,
            unsigned int row_count,
            unsigned int col_count)
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }
    MX_Xd_dynamic block(
            unsigned int row_start,
            unsigned int col_start,
            unsigned int row_count,
            unsigned int col_count) const
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_SubMatrix operator[](unsigned int i) {
        return (*this)(i);
    }
    MX_Xd_SubMatrix operator()(unsigned int i, unsigned int j=0) {
        return this->casadi::MX::operator()(
                    casadi::Slice(static_cast<casadi_int>(i), static_cast<casadi_int>(i+1)),
                    casadi::Slice(static_cast<casadi_int>(j), static_cast<casadi_int>(j+1)));
    }
    MX_Xd_scalar operator[](unsigned int i) const {
        return (*this)(i);
    }
    MX_Xd_scalar operator()(unsigned int i, unsigned int j=0) const {
        return this->casadi::MX::operator()(i, j);
    }


    MX_Xd_scalar dot(const MX_Xd_static<ncols, 1> &other_vector) const {
        return casadi::MX::dot(*this, other_vector);
    }

    MX_Xd_static<3, 1> cross(const MX_Xd_static<3, 1> &other_vector) const {
            MX_Xd_static<3, 1> result;
            result[0] = (*this)[1] * other_vector[2] - (*this)[2] * other_vector[1];
            result[1] = (*this)[2] * other_vector[0] - (*this)[0] * other_vector[2];
            result[2] = (*this)[0] * other_vector[1] - (*this)[1] * other_vector[0];

            return result;
    }

    MX_Xd_static<ncols, nrows> transpose() const {
        return T();
    }
    MX_Xd_static<ncols, nrows> inverse() const {
        if (ncols == 3 && nrows == 3){
            // computes the inverse of a matrix m
            MX_Xd_scalar det = (*this)(0, 0) * ((*this)(1, 1) * (*this)(2, 2) - (*this)(2, 1) * (*this)(1, 2)) -
                         (*this)(0, 1) * ((*this)(1, 0) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 0)) +
                         (*this)(0, 2) * ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0));

            MX_Xd_scalar invdet = 1 / det;

            MX_Xd_static<3, 3> minv; // inverse of matrix m
            minv(0, 0) = ((*this)(1, 1) * (*this)(2, 2) - (*this)(2, 1) * (*this)(1, 2)) * invdet;
            minv(0, 1) = ((*this)(0, 2) * (*this)(2, 1) - (*this)(0, 1) * (*this)(2, 2)) * invdet;
            minv(0, 2) = ((*this)(0, 1) * (*this)(1, 2) - (*this)(0, 2) * (*this)(1, 1)) * invdet;
            minv(1, 0) = ((*this)(1, 2) * (*this)(2, 0) - (*this)(1, 0) * (*this)(2, 2)) * invdet;
            minv(1, 1) = ((*this)(0, 0) * (*this)(2, 2) - (*this)(0, 2) * (*this)(2, 0)) * invdet;
            minv(1, 2) = ((*this)(1, 0) * (*this)(0, 2) - (*this)(0, 0) * (*this)(1, 2)) * invdet;
            minv(2, 0) = ((*this)(1, 0) * (*this)(2, 1) - (*this)(2, 0) * (*this)(1, 1)) * invdet;
            minv(2, 1) = ((*this)(2, 0) * (*this)(0, 1) - (*this)(0, 0) * (*this)(2, 1)) * invdet;
            minv(2, 2) = ((*this)(0, 0) * (*this)(1, 1) - (*this)(1, 0) * (*this)(0, 1)) * invdet;
            return minv;
        } else {
            return inv(*this);
        }
    }

    MX_Xd_scalar norm() const{
        return casadi::MX::norm_2(*this);
    }

    MX_Xd_scalar squaredNorm() const{
        return norm() * norm();
    }

    void normalize() {
        *this /= norm();
    }


    void operator+=(
            const MX_Xd_static<nrows, ncols>& other) {
        this->casadi::MX::operator+=(other);
    }
    MX_Xd_static<nrows, ncols> operator+(
            const MX_Xd_static<nrows, ncols>& other) const {
        MX_Xd_static<nrows, ncols> out(*this);
        return out.casadi::MX::operator+=(other);
    }
    void operator-=(
            const MX_Xd_static<nrows, ncols>& other) {
        this->casadi::MX::operator-=(other);
    }
    MX_Xd_static<nrows, ncols> operator-(
            const MX_Xd_static<nrows, ncols>& other) const {
        MX_Xd_static<nrows, ncols> out(*this);
        return out.casadi::MX::operator-=(other);
    }
    template <unsigned int ncols2>
    void operator*=(
            const MX_Xd_static<ncols, ncols2>& m2) {
        *this = casadi::MX::mtimes(*this, m2);
    }
    template <unsigned int ncols2>
    MX_Xd_static<nrows, ncols2> operator*(const MX_Xd_static<ncols, ncols2>& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    MX_Xd_static<nrows, ncols> operator*(
            const MX_Xd_scalar& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    void operator*=(
            const MX_Xd_scalar& other) {
        *this = casadi::MX::mtimes(*this, other);
    }

    void operator/=(const MX_Xd_scalar &scalar) {
        this->casadi::MX::operator/=(scalar(0, 0));
    }
    MX_Xd_static<nrows, ncols> operator/(const MX_Xd_scalar &scalar) const {
        MX_Xd_static<nrows, ncols> result (*this);
        for (unsigned int i = 0; i < nrows * ncols; i++)
            result[i] /= scalar;
        return result;
    }
    MX_Xd_static<nrows, ncols> operator/(
            const MX_Xd_SubMatrix& scalar) const {
        MX_Xd_static<nrows, ncols> result (*this);
        for (unsigned int i = 0; i < nrows * ncols; i++)
            result[i] /= scalar;
        return result;
    }
};

}

/* MX_XD_STATIC_H */
#endif

