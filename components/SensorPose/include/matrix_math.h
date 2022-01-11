#pragma once

#include <array>


std::array<std::array<float, 3>, 3> operator+(std::array<std::array<float, 3>, 3> mat1, std::array<std::array<float, 3>, 3> const& mat2);

std::array<std::array<float, 3>, 3> operator*(std::array<std::array<float, 3>, 3> const& mat1, std::array<std::array<float, 3>, 3> const& mat2);