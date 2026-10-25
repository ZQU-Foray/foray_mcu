#include "Libraries/Algorithm/alg_math/Matrix3.h"
#include <cstddef>
#include <limits>

namespace alg_math
{

namespace
{
// 与上游参考实现一致：用 float 的机器精度做奇异判据。
constexpr double Epsilon{static_cast<double>(std::numeric_limits<float>::epsilon())};
} // namespace

Matrix3 ScaledIdentity(double scale) noexcept
{
  Matrix3 result{};
  result.value[0] = scale;
  result.value[4] = scale;
  result.value[8] = scale;
  return result;
}

Matrix3 Multiply(const Matrix3 &first, const Matrix3 &second) noexcept
{
  Matrix3 result{};
  result.value[0] =
      first.value[0] * second.value[0] + first.value[1] * second.value[3] + first.value[2] * second.value[6];
  result.value[1] =
      first.value[0] * second.value[1] + first.value[1] * second.value[4] + first.value[2] * second.value[7];
  result.value[2] =
      first.value[0] * second.value[2] + first.value[1] * second.value[5] + first.value[2] * second.value[8];
  result.value[3] =
      first.value[3] * second.value[0] + first.value[4] * second.value[3] + first.value[5] * second.value[6];
  result.value[4] =
      first.value[3] * second.value[1] + first.value[4] * second.value[4] + first.value[5] * second.value[7];
  result.value[5] =
      first.value[3] * second.value[2] + first.value[4] * second.value[5] + first.value[5] * second.value[8];
  result.value[6] =
      first.value[6] * second.value[0] + first.value[7] * second.value[3] + first.value[8] * second.value[6];
  result.value[7] =
      first.value[6] * second.value[1] + first.value[7] * second.value[4] + first.value[8] * second.value[7];
  result.value[8] =
      first.value[6] * second.value[2] + first.value[7] * second.value[5] + first.value[8] * second.value[8];
  return result;
}

Matrix3 MultiplyTransposedFirst(const Matrix3 &first, const Matrix3 &second) noexcept
{
  Matrix3 result{};
  result.value[0] =
      first.value[0] * second.value[0] + first.value[3] * second.value[3] + first.value[6] * second.value[6];
  result.value[1] =
      first.value[0] * second.value[1] + first.value[3] * second.value[4] + first.value[6] * second.value[7];
  result.value[2] =
      first.value[0] * second.value[2] + first.value[3] * second.value[5] + first.value[6] * second.value[8];
  result.value[3] =
      first.value[1] * second.value[0] + first.value[4] * second.value[3] + first.value[7] * second.value[6];
  result.value[4] =
      first.value[1] * second.value[1] + first.value[4] * second.value[4] + first.value[7] * second.value[7];
  result.value[5] =
      first.value[1] * second.value[2] + first.value[4] * second.value[5] + first.value[7] * second.value[8];
  result.value[6] =
      first.value[2] * second.value[0] + first.value[5] * second.value[3] + first.value[8] * second.value[6];
  result.value[7] =
      first.value[2] * second.value[1] + first.value[5] * second.value[4] + first.value[8] * second.value[7];
  result.value[8] =
      first.value[2] * second.value[2] + first.value[5] * second.value[5] + first.value[8] * second.value[8];
  return result;
}

Matrix3 MultiplyTransposedSecond(const Matrix3 &first, const Matrix3 &second) noexcept
{
  Matrix3 result{};
  result.value[0] =
      first.value[0] * second.value[0] + first.value[1] * second.value[1] + first.value[2] * second.value[2];
  result.value[1] =
      first.value[0] * second.value[3] + first.value[1] * second.value[4] + first.value[2] * second.value[5];
  result.value[2] =
      first.value[0] * second.value[6] + first.value[1] * second.value[7] + first.value[2] * second.value[8];
  result.value[3] =
      first.value[3] * second.value[0] + first.value[4] * second.value[1] + first.value[5] * second.value[2];
  result.value[4] =
      first.value[3] * second.value[3] + first.value[4] * second.value[4] + first.value[5] * second.value[5];
  result.value[5] =
      first.value[3] * second.value[6] + first.value[4] * second.value[7] + first.value[5] * second.value[8];
  result.value[6] =
      first.value[6] * second.value[0] + first.value[7] * second.value[1] + first.value[8] * second.value[2];
  result.value[7] =
      first.value[6] * second.value[3] + first.value[7] * second.value[4] + first.value[8] * second.value[5];
  result.value[8] =
      first.value[6] * second.value[6] + first.value[7] * second.value[7] + first.value[8] * second.value[8];
  return result;
}

bool Inverted(const Matrix3 &input, Matrix3 &output) noexcept
{
  const double *in = input.value;
  const double a = in[4] * in[8] - in[5] * in[7];
  const double d = in[2] * in[7] - in[1] * in[8];
  const double g = in[1] * in[5] - in[2] * in[4];
  const double b = in[5] * in[6] - in[3] * in[8];
  const double e = in[0] * in[8] - in[2] * in[6];
  const double h = in[2] * in[3] - in[0] * in[5];
  const double c = in[3] * in[7] - in[4] * in[6];
  const double f = in[1] * in[6] - in[0] * in[7];
  const double i = in[0] * in[4] - in[1] * in[3];

  const double determinant = in[0] * a + in[1] * b + in[2] * c;
  if (determinant >= -Epsilon && determinant <= Epsilon)
  {
    for (std::size_t index = 0U; index < 9U; ++index)
    {
      output.value[index] = 0.0;
    }
    return false;
  }

  output.value[0] = a / determinant;
  output.value[1] = d / determinant;
  output.value[2] = g / determinant;
  output.value[3] = b / determinant;
  output.value[4] = e / determinant;
  output.value[5] = h / determinant;
  output.value[6] = c / determinant;
  output.value[7] = f / determinant;
  output.value[8] = i / determinant;
  return true;
}

void LimitInPlace(Matrix3 &value, double minimum, double maximum) noexcept
{
  for (std::size_t index = 0U; index < 9U; ++index)
  {
    if (value.value[index] < minimum)
    {
      value.value[index] = minimum;
    }
    else if (value.value[index] > maximum)
    {
      value.value[index] = maximum;
    }
  }
}

} // namespace alg_math
