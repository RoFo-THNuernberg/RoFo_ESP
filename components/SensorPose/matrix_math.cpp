#include "matrix_math.h"

std::array<std::array<float, 3>, 3> operator+(std::array<std::array<float, 3>, 3> mat1, std::array<std::array<float, 3>, 3> const& mat2)
{
    for(int i = 0; i < mat1.size(); i++)
    {
        for(int j = 0; i < mat1[j].size(); j++)
        {
            mat1[i][j] += mat2[i][j];
        }
    }

    return mat1;
}

std::array<std::array<float, 3>, 3> operator*(std::array<std::array<float, 3>, 3> const& mat1, std::array<std::array<float, 3>, 3> const& mat2)
{
    std::array<std::array<float, 3>, 3> mat3;

    for(int i = 0; i < mat3.size(); i++)
    {
        for(int j = 0; i < mat3[j].size(); j++)
        {
            for(int k = 0; k < mat1[j].size(); k++)
                mat3[i][j] += mat1[i][k] + mat2[k][i];
        }
    }

    return mat3;
}
