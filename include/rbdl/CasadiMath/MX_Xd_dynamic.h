/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef MX_XD_DYNAMICS_H
#define MX_XD_DYNAMICS_H

#include <vector>
#include <string>
#include <memory>

#include <casadi.hpp>
#include "MX_Xd_scalar.h"
#include "MX_Xd_subMatrix.h"

namespace RBDLCasadiMath {

class MX_Xd_dynamic : public casadi::MX{
public:
    MX_Xd_dynamic(
            unsigned int nrows = 1,
            unsigned int ncols = 1) : casadi::MX(nrows, ncols){
    }

    virtual ~MX_Xd_dynamic(){

    }


    MX_Xd_dynamic(const casadi::MX& m) :
        casadi::MX(m){
    }

    void conservativeResize (unsigned int nrows, unsigned int ncols = 1) {
        MX_Xd_dynamic result = casadi::MX::zeros(nrows, ncols);

        unsigned int arows = std::min (nrows, rows());
        unsigned int acols = std::min (ncols, cols());

        for (unsigned int i = 0; i < arows; i++) {
            for (unsigned int j = 0; j < acols; j++) {
                result(i,j) = (*this)(i,j);
            }
        }

        *this = result;
    }

    static MX_Xd_dynamic Zero(unsigned int nrows, unsigned int ncols = 1){
        return casadi::MX::zeros(nrows, ncols);
    }
    MX_Xd_dynamic& setZero(){
        *this = casadi::MX::zeros(this->rows(), this->cols());
        return *this;
    }

    static MX_Xd_dynamic One(unsigned int nrows, unsigned int ncols = 1){
        return casadi::MX::ones(nrows, ncols);
    }
    MX_Xd_dynamic& setOnes(){
        *this = casadi::MX::ones(this->rows(), this->cols());
        return *this;
    }

    static MX_Xd_dynamic Identity(unsigned int size, unsigned int ignoredSize = 0){
        return casadi::MX::eye(size);
    }
    MX_Xd_dynamic& setIdentity(){
        *this = casadi::MX::eye(cols());
        return *this;
    }

    MX_Xd_SubMatrix operator[](unsigned int i) {
        return (*this)(i, 0);
    }
    MX_Xd_scalar operator[](unsigned int i) const {
        return (*this)(i, 0);
    }
    MX_Xd_SubMatrix operator()(unsigned int i, unsigned int j=0) {
        return this->casadi::MX::operator()(
                    casadi::Slice(static_cast<casadi_int>(i), static_cast<casadi_int>(i+1)),
                    casadi::Slice(static_cast<casadi_int>(j), static_cast<casadi_int>(j+1)));
    }
    MX_Xd_scalar operator()(unsigned int i, unsigned int j=0) const {
        return this->casadi::MX::operator()(i, j);
    }

    void resize(unsigned int newI, unsigned int newJ = 1){
        *this = casadi::MX(newI, newJ);
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
    MX_Xd_SubMatrix block(
            unsigned int row_start,
            unsigned int col_start)
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }
    template <unsigned int row_count, unsigned int col_count>
    MX_Xd_dynamic block(
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

    MX_Xd_SubMatrix topRows(
            unsigned int numberRows)
    {
        return (*this).block(0, 0, numberRows, cols());
    }
    MX_Xd_dynamic topRows(
            unsigned int numberRows) const
    {
        return (*this).block(0, 0, numberRows, cols());
    }


    MX_Xd_dynamic transpose() const {
        return T();
    }

    MX_Xd_dynamic inverse() const {
        return inv(*this);
    }

    MX_Xd_scalar dot(const MX_Xd_dynamic &other_vector) const {
        return casadi::MX::dot(*this, other_vector);
    }

    MX_Xd_dynamic norm() const {
        return casadi::MX::norm_2(*this);
    }

    void normalize() {
        *this /= norm();
    }

    MX_Xd_dynamic squaredNorm() const {
        return norm() * norm();
    }

    void operator+=(
            const MX_Xd_dynamic& other) {
        this->casadi::MX::operator+=(other);
    }
    MX_Xd_dynamic operator+(
            const MX_Xd_dynamic& other) const {
        MX_Xd_dynamic out(*this);
        return out.casadi::MX::operator+=(other);
    }
    void operator-=(
            const MX_Xd_dynamic& other) {
        this->casadi::MX::operator-=(other);
    }
    MX_Xd_dynamic operator-(
            const MX_Xd_dynamic& other) const {
        MX_Xd_dynamic out(*this);
        return out.casadi::MX::operator-=(other);
    }
    void operator*=(
            const MX_Xd_dynamic& m2) {
        *this = casadi::MX::mtimes(*this, m2);
    }
    MX_Xd_dynamic operator*(const MX_Xd_dynamic& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    MX_Xd_dynamic operator*(const MX_Xd_scalar& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    MX_Xd_dynamic operator*(const MX_Xd_SubMatrix& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    MX_Xd_dynamic operator*(const double& other) const {
        return casadi::MX::mtimes(*this, other);
    }

    void operator/=(const MX_Xd_scalar &scalar) {
        for (unsigned int i = 0; i < rows() * cols(); i++)
            this->casadi::MX::operator/=(scalar(0, 0));
    }
    MX_Xd_dynamic operator/(const MX_Xd_scalar &scalar) const {
        MX_Xd_dynamic result (*this);
        for (unsigned int i = 0; i < rows(); ++i){
            for (unsigned int j = 0; j < cols(); ++j){
                result(i, j) /= scalar;
            }
        }
        return result;
    }
    MX_Xd_dynamic operator/(
            const MX_Xd_SubMatrix& scalar) const {
        MX_Xd_dynamic result (*this);
        for (unsigned int i = 0; i < rows(); ++i){
            for (unsigned int j = 0; j < cols(); ++j){
                result(i, j) /= scalar;
            }
        }
        return result;
    }
};

}

/* MX_XD_DYNAMICS_H */
#endif

