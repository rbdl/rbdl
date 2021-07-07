/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef MX_XD_SCALAR_H
#define MX_XD_SCALAR_H

#include <vector>
#include <string>
#include <memory>

#include <casadi.hpp>

namespace RBDLCasadiMath {

class MX_Xd_scalar : public casadi::MX{
public:
    MX_Xd_scalar() : casadi::MX(1, 1){

    }

    virtual ~MX_Xd_scalar(){

    }


    MX_Xd_scalar(const double val) : casadi::MX(1, 1){
        (*this)(0, 0) = val;
    }

    MX_Xd_scalar(const casadi::MX& m) : casadi::MX(m){

    }

    MX_Xd_scalar(
            const MX_Xd_scalar& v0) :
        casadi::MX(1, 1)
    {
        (*this)(0) = v0(0);
    }

    unsigned int rows() const {
        return 1;
    }

    unsigned int cols() const {
        return 1;
    }

    unsigned int size() const {
        return 1;
    }

    MX_Xd_scalar operator[](unsigned int i) const{
        return (*this)(i);
    }

    void operator+=(
            const MX_Xd_scalar& other) {
        this->casadi::MX::operator+=(other);
    }
    MX_Xd_scalar operator+(
            const MX_Xd_scalar& other) const {
        MX_Xd_scalar out(*this);
        return out.casadi::MX::operator+=(other);
    }

    void operator-=(
            const MX_Xd_scalar& other) {
        this->casadi::MX::operator-=(other);
    }
    MX_Xd_scalar operator-(
            const MX_Xd_scalar& other) const {
        MX_Xd_scalar out(*this);
        return out.casadi::MX::operator-=(other);
    }
    MX_Xd_scalar operator-(
            const double& other) const {
        MX_Xd_scalar out(*this);
        return out.casadi::MX::operator-=(other);
    }

    MX_Xd_scalar operator*(const MX_Xd_scalar& other) const{
        return casadi::MX::times(*this, other);
    }
    MX_Xd_scalar operator*(
            double other) {
        return casadi::MX::times(*this, other);
    }

    void operator/=(
            const MX_Xd_scalar& other
            ){
        this->casadi::MX::operator/=(other);
    }
    MX_Xd_scalar operator/(
            const MX_Xd_scalar& other) const {
        MX_Xd_scalar out(*this);
        return out.casadi::MX::operator/=(other);
    }
};

}

/* MX_XD_SCALAR_H */
#endif

