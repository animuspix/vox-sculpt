export module vmath;

import <concepts>;
import <cmath>;
import <cassert>;
import <type_traits>;

//#define MATH_DBG
#ifdef MATH_DBG
#pragma optimize("", off)
#endif
export namespace vmath
{
	// Useful constants
	constexpr float pi = 3.14159265358979323846f;
	constexpr float inv_pi = 1.0f / pi;
	constexpr float half_pi = 0.5f * pi;
	constexpr float quarter_pi = 0.25f * pi;
	constexpr float pi_2 = 2.0f * pi;
	constexpr float eps = 0.000001f;

	// Constraint for supported vector element types
	// No support for vectors of vectors (tensors?) yet - I'll add them once I need them
	template<typename numeric_t>
	concept vec_elt_type = std::is_floating_point_v<numeric_t> ||
		std::is_integral_v<numeric_t>;

	// Linear algebra classes
	template<int dim, vec_elt_type type = float> requires (dim < 5 && dim > 1)
		struct vec
	{
		using vec_type = type;
		vec<dim, type>()
		{
			for (int i = 0; i < dim; i++)
			{
				e[i] = 0;
			}
		}
		template<vec_elt_type scalar_t>
		vec<dim, type>(scalar_t _x)
		{
			// Valid for all dimensions - behaves like a broadcast
			for (int i = 0; i < dim; i++)
			{
				e[i] = static_cast<type>(_x);
			}
		}
		template<vec_elt_type x_t, vec_elt_type y_t>
		vec<dim, type>(x_t _x, y_t _y)
		{
			static_assert(dim == 2, "two-dimensional constructor used with higher-dimensional vector");
			e[0] = static_cast<type>(_x);
			e[1] = static_cast<type>(_y);
		}
		template<vec_elt_type x_t, vec_elt_type y_t, vec_elt_type z_t>
		vec<dim, type>(x_t _x, y_t _y, z_t _z)
		{
			static_assert(dim == 3, "three-dimensional constructor used with differently-dimensional vector");
			e[0] = static_cast<type>(_x);
			e[1] = static_cast<type>(_y);
			e[2] = static_cast<type>(_z);
		}
		template<vec_elt_type x_t, vec_elt_type y_t, vec_elt_type z_t, vec_elt_type w_t>
		vec<dim, type>(x_t _x, y_t _y, z_t _z, w_t _w)
		{
			static_assert(dim == 4, "four-dimensional constructor used with lower-dimensional vector");
			e[0] = static_cast<type>(_x);
			e[1] = static_cast<type>(_y);
			e[2] = static_cast<type>(_z);
			e[3] = static_cast<type>(_w);
		}
		type e[dim] = {};
		float dot(vec<dim, type> u)
		{
			float ret = 0;
			for (int i = 0; i < dim; i++)
			{
				ret += u.e[i] * e[i];
			}
			return ret;
		}
		vec<3, type> cross(vec<3, type> u)
		{
			static_assert(dim == 3, "The cross product is only meaningful in three dimensions - the wedge product is interesting & more general but i haven't found a use-case for it here yet");
			vec<3> leftMuls(e[1] * u.e[2],
				e[2] * u.e[0],
				e[0] * u.e[1]);
			vec<3> rightMuls(e[2] * u.e[1],
				e[0] * u.e[2],
				e[1] * u.e[0]);
			return vec<3>(leftMuls.e[0] - rightMuls.e[0],
				leftMuls.e[1] - rightMuls.e[1],
				leftMuls.e[2] - rightMuls.e[2]);
		}

		// Need to refresh my memory of how this math works...
		vec<4, type> qtn_rotation_concat(vec<4, type> q)
		{
			static_assert(dim == 4, "Quaternionic vectors must have four axes");
			vmath::vec<3> v = xyz();
			vec<3> a = w() * q.xyz();
			vec<3> b = q.w() * v;
			vec<3> ortho = a.cross(b);
			vec<3> u = a + b + ortho;
			return vec<4>(u.x(), u.y(), u.z(),
				(w() * q.w()) - v.dot(q.xyz()));
		}
		vec<3, type> qtn_rotation_apply(vec<4, type> q)
		{
			static_assert(dim == 3, "This function is for applying quaternionic rotations to 3D vectors - 2D vectors can be rotated with regular sin/cos, and quaternion rotations can be concatenated using [qtn_rotation_concat] (see above)");
			vec<4> p = vec<4>(x(), y(), z(), 0.0);
			vec<4> qv = p.qtn_rotation_concat(q);
			vec<4> q_inv = vec<4>(qv.x() * -1.0f,
				qv.y() * -1.0f,
				qv.z() * -1.0f, q.w());
			return qv.qtn_rotation_concat(q_inv).xyz();
		}
		const type sqr_magnitude() const
		{
			type acc = 0;
			for (int i = 0; i < dim; i++)
			{
				acc += e[i] * e[i];
			}
			return acc;
		}
		const type magnitude() const
		{
			type acc = 0;
			for (int i = 0; i < dim; i++)
			{
				acc += e[i] * e[i];
			}
			return static_cast<type>(std::sqrt(float(acc))); // May lose accuracy for integer vectors
		}
		vec<dim, type> normalized() const
		{
			type len = magnitude();
			vec<dim> d = *this;
			for (int i = 0; i < dim; i++) d.e[i] /= len; // May lose accuracy for integer vectors
			return d;
		}
		const type x() const
		{
			return e[0];
		}
		const type y() const
		{
			return e[1];
		}
		const type z() const
		{
			static_assert(dim >= 3, "no z-coordinate available");
			return e[2];
		}
		const type w() const
		{
			static_assert(dim == 4, "no w-coordinate available");
			return e[3];
		}
		const vec<2, type> xy() const
		{
			return vec<2, type>(e[0], e[1]);
		}
		const vec<2, type> xz() const
		{
			static_assert(dim >= 3, "no z-coordinate available");
			return vec<2>(e[0], e[2]);
		}
		const vec<2, type> zw() const
		{
			static_assert(dim == 4, "no w-coordinate available");
			return vec<2>(e[2], e[3]);
		}
		const vec<2, type> yw() const
		{
			static_assert(dim == 4, "no w-coordinate available");
			return vec<2>(e[1], e[3]);
		}
		const vec<3, type> xyz() const
		{
			static_assert(dim >= 3, "Not enough components for XYZ swizzle");
			return vec<3>(e[0], e[1], e[2]);
		}
	};
	struct m3
	{
		m3(vec<3> _X, vec<3> _Y, vec<3> _Z) : X(_X), Y(_Y), Z(_Z) {}
		m3(float x0, float y0, float z0,
			float x1, float y1, float z1,
			float x2, float y2, float z2) :
			X(vec<3>(x0, y0, z0)),
			Y(vec<3>(x1, y1, z1)),
			Z(vec<3>(x2, y2, z2)) {}
		vec<3> X, Y, Z;
		m3 transposed()
		{
			vec<3> _X(X.e[0], Y.e[0], Z.e[0]);
			vec<3> _Y(X.e[1], Y.e[1], Z.e[1]);
			vec<3> _Z(X.e[2], Y.e[2], Z.e[2]);
			return m3(_X, _Y, _Z);
		}
		vec<3> apply(vec<3> v) // matrix x vector product
		{
			const vec<3> xT(X.x(), Y.x(), Z.x()); // Implicit transpose, we have to be computing rows x columns and not rows x rows
			const vec<3> yT(X.y(), Y.y(), Z.y());
			const vec<3> zT(X.z(), Y.z(), Z.z());
			return vec<3>(v.dot(xT),
				v.dot(yT),
				v.dot(zT));
		}
		m3 chain_with(m3 m) // matrix x matrix proeduct for chained transformations
		{
			m3 rhsT = m.transposed();
			return m3(vec<3>(X.dot(rhsT.X),
				X.dot(rhsT.Y),
				X.dot(rhsT.Z)),
				vec<3>(Y.dot(rhsT.X),
					Y.dot(rhsT.Y),
					Y.dot(rhsT.Z)),
				vec<3>(Z.dot(rhsT.X),
					Z.dot(rhsT.Y),
					Z.dot(rhsT.Z)));
		}
	};

	// Type constraints for vox_sculpt maths functions
	template<typename vector_t>
	concept vec2_type_fp = std::is_same_v<vector_t, vec<2, float>> ||
		std::is_same_v<vector_t, vec<2, double>>;

	template<typename vector_t>
	concept vec3_type_fp = std::is_same_v<vector_t, vec<3, float>> ||
		std::is_same_v<vector_t, vec<3, double>>;

	template<typename vector_t>
	concept vec4_type_fp = std::is_same_v<vector_t, vec<4, float>> ||
		std::is_same_v<vector_t, vec<4, double>>;

	template<typename vector_t>
	concept vec2_type_integral = std::is_same_v<vector_t, vec<2, uint8_t>> ||
		std::is_same_v<vector_t, vec<2, uint16_t>> ||
		std::is_same_v<vector_t, vec<2, uint32_t>> ||
		std::is_same_v<vector_t, vec<2, uint64_t>> ||
		std::is_same_v<vector_t, vec<2, int8_t>> ||
		std::is_same_v<vector_t, vec<2, int16_t>> ||
		std::is_same_v<vector_t, vec<2, int32_t>> ||
		std::is_same_v<vector_t, vec<2, int64_t>>;

	template<typename vector_t>
	concept vec3_type_integral = std::is_same_v<vector_t, vec<3, uint8_t>> ||
		std::is_same_v<vector_t, vec<3, uint16_t>> ||
		std::is_same_v<vector_t, vec<3, uint32_t>> ||
		std::is_same_v<vector_t, vec<3, uint64_t>> ||
		std::is_same_v<vector_t, vec<3, int8_t>> ||
		std::is_same_v<vector_t, vec<3, int16_t>> ||
		std::is_same_v<vector_t, vec<3, int32_t>> ||
		std::is_same_v<vector_t, vec<3, int64_t>>;

	template<typename vector_t>
	concept vec4_type_integral = std::is_same_v<vector_t, vec<4, uint8_t>> ||
		std::is_same_v<vector_t, vec<4, uint16_t>> ||
		std::is_same_v<vector_t, vec<4, uint32_t>> ||
		std::is_same_v<vector_t, vec<4, uint64_t>> ||
		std::is_same_v<vector_t, vec<4, int8_t>> ||
		std::is_same_v<vector_t, vec<4, int16_t>> ||
		std::is_same_v<vector_t, vec<4, int32_t>> ||
		std::is_same_v<vector_t, vec<4, int64_t>>;

	template<typename vector_t>
	concept vec2_type = vec2_type_fp<vector_t> || vec2_type_integral<vector_t>;

	template<typename vector_t>
	concept vec3_type = vec3_type_fp<vector_t> || vec3_type_integral<vector_t>;

	template<typename vector_t>
	concept vec4_type = vec4_type_fp<vector_t> || vec4_type_integral<vector_t>;

	template <typename scalar_numeric_t>
	concept scalar_maths_type = std::is_floating_point_v<scalar_numeric_t> ||
		std::is_integral_v<scalar_numeric_t>;

	template<typename numeric_t>
	concept maths_type = std::is_floating_point_v<numeric_t> ||
		std::is_integral_v<numeric_t> ||
		vec2_type<numeric_t> ||
		vec3_type<numeric_t> ||
		vec4_type<numeric_t>;

	template<typename numeric_t>
	concept maths_type_fp = std::is_floating_point_v<numeric_t> ||
		vec2_type_fp<numeric_t> ||
		vec3_type_fp<numeric_t> ||
		vec4_type_fp<numeric_t>;

	template<typename numeric_t>
	concept maths_type_integral = std::is_integral_v<numeric_t> ||
		vec2_type_integral<numeric_t> ||
		vec3_type_integral<numeric_t> ||
		vec4_type_integral<numeric_t>;

	// Linear algebra operators
	// 3D
	template<vec3_type vec3>
	const vec3 operator+(vec3 lhs, vec3 rhs)
	{
		return vec3(lhs.e[0] + rhs.e[0],
			lhs.e[1] + rhs.e[1],
			lhs.e[2] + rhs.e[2]);
	}

	template<vec3_type vec3>
	const vec3 operator+=(vec3& lhs, vec3 rhs)
	{
		lhs = vec3(lhs.e[0] + rhs.e[0],
			lhs.e[1] + rhs.e[1],
			lhs.e[2] + rhs.e[2]);
		return lhs;
	}

	template<vec3_type vec3>
	const vec3 operator-(vec3 lhs, vec3 rhs)
	{
		return vec3(lhs.e[0] - rhs.e[0],
			lhs.e[1] - rhs.e[1],
			lhs.e[2] - rhs.e[2]);
	}
	template<vec3_type vec3>
	const vec3 operator-=(vec3& lhs, vec3 rhs)
	{
		lhs = lhs - rhs;
		return lhs;
	}
	template<vec3_type vec3>
	const vec3 operator*(vec3 lhs, vec3 rhs)
	{
		return vec3(lhs.e[0] * rhs.e[0],
			lhs.e[1] * rhs.e[1],
			lhs.e[2] * rhs.e[2]);
	}
	template<vec3_type vec3>
	const vec3 operator*=(vec3& lhs, vec3 rhs)
	{
		lhs = lhs * rhs;
		return lhs;
	}

	template<vec3_type vec3>
	const vec3 operator*(vec3 lhs, float rhs)
	{
		return vec3(lhs.e[0] * rhs,
			lhs.e[1] * rhs,
			lhs.e[2] * rhs);
	}

	template<vec3_type vec3>
	const vec3 operator*(float lhs, vec3 rhs)
	{
		return vec3(rhs.e[0] * lhs,
			rhs.e[1] * lhs,
			rhs.e[2] * lhs);
	}

	template<vec3_type vec3>
	const vec3 operator*=(vec3& lhs, float rhs)
	{
		lhs = lhs * rhs;
		return lhs;
	}

	template<vec3_type vec3>
	const vec3 operator/(vec3 lhs, vec3 rhs)
	{
		return vec3(lhs.e[0] / rhs.e[0],
			lhs.e[1] / rhs.e[1],
			lhs.e[2] / rhs.e[2]);
	}

	template<vec3_type vec3>
	const vec3 operator/=(vec3& lhs, vec3 rhs)
	{
		lhs = lhs / rhs;
		return lhs;
	}

	template<vec3_type vec3>
	const vec3 operator/(vec3 lhs, float rhs)
	{
		return vec3(lhs.e[0] / rhs,
			lhs.e[1] / rhs,
			lhs.e[2] / rhs);
	}

	template<vec3_type vec3>
	const vec3 operator/=(vec3& lhs, float rhs)
	{
		lhs = lhs / rhs;
		return lhs;
	}

	template<vec3_type vec3>
	const bool anyGreaterElements(vec3 lhs, vec3 rhs)
	{
		return lhs.e[0] > rhs.e[0] ||
			lhs.e[1] > rhs.e[1] ||
			lhs.e[2] > rhs.e[2];
	}

	template<vec3_type vec3>
	const bool anyLesserElements(vec3 lhs, vec3 rhs)
	{
		return lhs.e[0] < rhs.e[0] ||
			lhs.e[1] < rhs.e[1] ||
			lhs.e[2] < rhs.e[2];
	}

	template<vec3_type vec3>
	const bool anyEqualElements(vec3 lhs, vec3 rhs)
	{
		return lhs.e[0] == rhs.e[0] ||
			lhs.e[1] == rhs.e[1] ||
			lhs.e[2] == rhs.e[2];
	}

	template<vec3_type vec3>
	const bool allGreaterElements(vec3 lhs, vec3 rhs)
	{
		return lhs.e[0] > rhs.e[0] &&
			lhs.e[1] > rhs.e[1] &&
			lhs.e[2] > rhs.e[2];
	}

	template<vec3_type vec3>
	const bool allLesserElements(vec3 lhs, vec3 rhs)
	{
		return lhs.e[0] < rhs.e[0] &&
			lhs.e[1] < rhs.e[1] &&
			lhs.e[2] < rhs.e[2];
	}

	template<vec3_type vec3>
	const bool allEqualElements(vec3 lhs, vec3 rhs)
	{
		return lhs.e[0] == rhs.e[0] &&
			lhs.e[1] == rhs.e[1] &&
			lhs.e[2] == rhs.e[2];
	}

	template<vec3_type vec3, scalar_maths_type scalar>
	const bool anyGreater(vec3 lhs, scalar rhs)
	{
		using comparison_type = std::common_type_t<typename vec3::vec_type, scalar>;
		const comparison_type comparison_rhs = static_cast<comparison_type>(rhs);
		return static_cast<comparison_type>(lhs.e[0]) > comparison_rhs ||
			static_cast<comparison_type>(lhs.e[1]) > comparison_rhs ||
			static_cast<comparison_type>(lhs.e[2]) > comparison_rhs;
	}

	template<vec3_type vec3, scalar_maths_type scalar>
	const bool anyLesser(vec3 lhs, scalar rhs)
	{
		using comparison_type = std::common_type_t<typename vec3::vec_type, scalar>;
		const comparison_type comparison_rhs = static_cast<comparison_type>(rhs);
		return static_cast<comparison_type>(lhs.e[0]) < comparison_rhs ||
			static_cast<comparison_type>(lhs.e[1]) < comparison_rhs ||
			static_cast<comparison_type>(lhs.e[2]) < comparison_rhs;
	}

	template<vec3_type vec3, scalar_maths_type scalar>
	const bool anyEqual(vec3 lhs, scalar rhs)
	{
		using comparison_type = std::common_type_t<typename vec3::vec_type, scalar>;
		const comparison_type comparison_rhs = static_cast<comparison_type>(rhs);
		return static_cast<comparison_type>(lhs.e[0]) == comparison_rhs ||
			static_cast<comparison_type>(lhs.e[1]) == comparison_rhs ||
			static_cast<comparison_type>(lhs.e[2]) == comparison_rhs;
	}

	template<vec3_type vec3, scalar_maths_type scalar>
	const bool allGreater(vec3 lhs, scalar rhs)
	{
		using comparison_type = std::common_type_t<typename vec3::vec_type, scalar>;
		const comparison_type comparison_rhs = static_cast<comparison_type>(rhs);
		return static_cast<comparison_type>(lhs.e[0]) > comparison_rhs &&
			static_cast<comparison_type>(lhs.e[1]) > comparison_rhs &&
			static_cast<comparison_type>(lhs.e[2]) > comparison_rhs;
	}

	template<vec3_type vec3, scalar_maths_type scalar>
	const bool allLesser(vec3 lhs, scalar rhs)
	{
		using comparison_type = std::common_type_t<typename vec3::vec_type, scalar>;
		const comparison_type comparison_rhs = static_cast<comparison_type>(rhs);
		return static_cast<comparison_type>(lhs.e[0]) < comparison_rhs &&
			static_cast<comparison_type>(lhs.e[1]) < comparison_rhs &&
			static_cast<comparison_type>(lhs.e[2]) < comparison_rhs;
	}

	template<vec3_type vec3, scalar_maths_type scalar>
	const bool allEqual(vec3 lhs, scalar rhs)
	{
		using comparison_type = std::common_type_t<typename vec3::vec_type, scalar>;
		const comparison_type comparison_rhs = static_cast<comparison_type>(rhs);
		return static_cast<comparison_type>(lhs.e[0]) == comparison_rhs &&
			static_cast<comparison_type>(lhs.e[1]) == comparison_rhs &&
			static_cast<comparison_type>(lhs.e[2]) == comparison_rhs;
	}

	// 2D
	template<vec2_type vec2>
	const vec2 operator+(vec2 lhs, vec2 rhs)
	{
		return vec2(lhs.e[0] + rhs.e[0],
			lhs.e[1] + rhs.e[1]);
	}

	template<vec2_type vec2>
	const vec2 operator+=(vec2& lhs, vec2 rhs)
	{
		lhs = vec2(lhs.e[0] + rhs.e[0],
			lhs.e[1] + rhs.e[1]);
		return lhs;
	}

	template<vec2_type vec2>
	const vec2 operator-(vec2 lhs, vec2 rhs)
	{
		return vec2(lhs.e[0] - rhs.e[0],
			lhs.e[1] - rhs.e[1]);
	}

	template<vec2_type vec2>
	const vec2 operator-=(vec2& lhs, vec2 rhs)
	{
		lhs = lhs - rhs;
		return lhs;
	}

	template<vec2_type vec2>
	const vec2 operator*(vec2 lhs, vec2 rhs)
	{
		return vec2(lhs.e[0] * rhs.e[0],
			lhs.e[1] * rhs.e[1]);
	}

	template<vec2_type vec2>
	const vec2 operator*=(vec2& lhs, vec2 rhs)
	{
		lhs = lhs * rhs;
		return lhs;
	}

	template<vec2_type vec2>
	const vec2 operator*(vec2 lhs, float rhs)
	{
		return vec2(lhs.e[0] * rhs,
			lhs.e[1] * rhs);
	}

	template<vec2_type vec2>
	const vec2 operator*(float lhs, vec2 rhs)
	{
		return vec2(rhs.e[0] * lhs,
			rhs.e[1] * lhs);
	}

	template<vec2_type vec2>
	const vec2 operator*=(vec2& lhs, float rhs)
	{
		lhs = lhs * rhs;
		return lhs;
	}

	template<vec2_type vec2>
	const vec2 operator/(vec2 lhs, vec2 rhs)
	{
		return vec2(lhs.e[0] / rhs.e[0],
			lhs.e[1] / rhs.e[1]);
	}

	template<vec2_type vec2>
	const vec2 operator/=(vec2& lhs, vec2 rhs)
	{
		lhs = lhs / rhs;
		return lhs;
	}

	template<vec2_type vec2>
	const vec2 operator/(vec2 lhs, float rhs)
	{
		return vec2(lhs.e[0] / rhs,
			lhs.e[1] / rhs);
	}

	template<vec2_type vec2>
	const vec2 operator/=(vec2& lhs, float rhs)
	{
		lhs = lhs / rhs;
		return lhs;
	}

	template<vec2_type vec2>
	const bool anyGreater(vec2 lhs, vec2 rhs)
	{
		return lhs.e[0] > rhs.e[0] ||
			lhs.e[1] > rhs.e[1];
	}

	template<vec2_type vec2>
	const bool anyLesser(vec2 lhs, vec2 rhs)
	{
		return lhs.e[0] < rhs.e[0] ||
			lhs.e[1] < rhs.e[1];
	}

	template<vec2_type vec2>
	const bool anyEqual(vec2 lhs, vec2 rhs)
	{
		return lhs.e[0] == rhs.e[0] ||
			lhs.e[1] == rhs.e[1];
	}

	template<vec2_type vec2>
	const bool allGreater(vec2 lhs, vec2 rhs)
	{
		return lhs.e[0] > rhs.e[0] &&
			lhs.e[1] > rhs.e[1];
	}

	template<vec2_type vec2>
	const bool allLesser(vec2 lhs, vec2 rhs)
	{
		return lhs.e[0] < rhs.e[0] &&
			lhs.e[1] < rhs.e[1];
	}

	template<vec2_type vec2>
	const bool allEqual(vec2 lhs, vec2 rhs)
	{
		return lhs.e[0] == rhs.e[0] &&
			lhs.e[1] == rhs.e[1];
	}

	template<vec2_type vec2>
	const bool anyGreater(vec2 lhs, float rhs)
	{
		return lhs.e[0] > rhs &&
			lhs.e[1] > rhs;
	}

	template<vec2_type vec2>
	const bool anyLesser(vec2 lhs, float rhs)
	{
		return lhs.e[0] < rhs &&
			lhs.e[1] < rhs;
	}

	template<vec2_type vec2>
	const bool anyEqual(vec2 lhs, float rhs)
	{
		return lhs.e[0] == rhs &&
			lhs.e[1] == rhs;
	}

	template<vec2_type vec2>
	const bool allGreater(vec2 lhs, float rhs)
	{
		return lhs.e[0] > rhs &&
			lhs.e[1] > rhs;
	}

	template<vec2_type vec2>
	const bool allLesser(vec2 lhs, float rhs)
	{
		return lhs.e[0] < rhs &&
			lhs.e[1] < rhs;
	}

	template<vec2_type vec2>
	const bool allEqual(vec2 lhs, float rhs)
	{
		return lhs.e[0] == rhs &&
			lhs.e[1] == rhs;
	}

	template<vec2_type_fp vec2>
	vec2 sin(vec2 v)
	{
		return vec2(std::sin(v.x()),
			std::sin(v.y()));
	}

	template<vec3_type_fp vec3>
	vec3 sin(vec3 v)
	{
		return vec3(std::sin(v.x()),
			std::sin(v.y()),
			std::sin(v.z()));
	}

	template<vec2_type_fp vec2>
	vec2 cos(vec2 v)
	{
		return vec2(
			std::cos(v.x()),
			std::cos(v.y()));
	}

	template<vec3_type_fp vec3>
	vec3 cos(vec3 v)
	{
		return vec3(
			std::cos(v.x()),
			std::cos(v.y()),
			std::cos(v.z()));
	}

	template<vec2_type_fp vec2>
	vec2 tan(vec2 v)
	{
		return vec2(
			std::tan(v.x()),
			std::tan(v.y()));
	}

	template<vec3_type_fp vec3>
	vec3 tan(vec3 v)
	{
		return vec3(
			std::tan(v.x()),
			std::tan(v.y()),
			std::tan(v.z()));
	}

	// General functions
	float polyMulBinomial(float x, float a, float b)
	{
		// (ax [operator] b)^2
		// quick, lazy FOIL polynomial expansion
		float ax = a * x;
		float axSqr = ax * ax;
		float axb_2 = (ax * b) * 2.0f;
		float bSqr = b * b;

		// (ax - b)^2 should expand to
		// ax^2 - 2axb + b^2
		return axSqr - axb_2 + bSqr;
	}

	float quadratic(float x, float a, float b, float c, bool inv = true)
	{
		// Solve inner multiply
		float polyNum = polyMulBinomial(x, a, b);

		// Invert, apply translation, return :)
		return inv ? -polyNum + c :
			polyNum + c;
	};

	float gaussian(float x, float a, float b, float c, float d)
	{
		// Solve inner polynomial
		float polyNum = polyMulBinomial(x, 1.0f, b);

		// Central fraction (stretch the polynomial + invert it)
		float frac = -(polyNum / (2.0f * c * c));

		// Unit gaussian
		float gauss = std::exp(frac);

		// Scale + translation
		return (a * gauss) - d;
	}

	m3 normalSpace(vec<3> n)
	{
		// Tangent normal function from:
		// https://www.shadertoy.com/view/ltKyzm
		// Originally from Scratchapixel
		// https://www.scratchapixel.com/lessons/3d-basic-rendering/global-illumination-path-tracing/global-illumination-path-tracing-practical-implementation

		// Generate a basis-x direction
		// Different functions will be more/less appropriate
		// depending on the normal's x/y-tendency
		vec<3> absNormal = vec<3>(std::abs(n.x()), std::abs(n.y()), std::abs(n.z()));
		vec<3> basisX = vec<3>(0, 0, 0);
		if (absNormal.x() >= absNormal.y())
		{
			basisX = vmath::vec<3>(n.z(), 0.0f, n.x() * -1.0f);
		}
		else
		{
			basisX = vmath::vec<3>(0.0f, n.z(), n.y() * -1.0f);
		}

		// Build the normal-space from the normal vector and the x-basis we generated before,
		// then return the result
		return m3(basisX,
			n,
			basisX.cross(n));
	}

	template<vec3_type_fp vec3>
	vec3 vfloor(vec3 v) // Integer vectors are floored by nature
	{
		return vec3(
			std::floor(v.e[0]),
			std::floor(v.e[1]),
			std::floor(v.e[2]));
	}

	template<maths_type_fp lerp_type>
	lerp_type lerp(lerp_type a, lerp_type b, lerp_type t) // Because I keep forgetting how these work: https://en.wikipedia.org/wiki/Linear_interpolation
	{
		return a + t * (a - b);
	}

	template<typename argType> requires (std::is_integral_v<argType> || std::is_floating_point_v<argType>)
		argType max(argType a, argType b)
	{
		return a > b ? a : b;
	}

	template<typename argType> requires (std::is_integral_v<argType> || std::is_floating_point_v<argType>)
		argType min(argType a, argType b)
	{
		return a > b ? b : a;
	}

	template<vec3_type vec3>
	vec3 vmax(vec3 v, vec3 u)
	{
		return vec3(max(v.e[0], u.e[0]),
			max(v.e[1], u.e[1]),
			max(v.e[2], u.e[2]));
	}

	template<vec3_type vec3>
	vec3 vmin(vec3 v, vec3 u)
	{
		return vec3(min(v.e[0], u.e[0]),
			min(v.e[1], u.e[1]),
			min(v.e[2], u.e[2]));
	}

	template<maths_type clamp_type>
	clamp_type clamp(clamp_type a, clamp_type clampMin, clamp_type clampMax)
	{
		constexpr bool scalar = std::is_integral_v<clamp_type> || std::is_floating_point_v<clamp_type>;
		if constexpr (scalar)
		{
			return min(max(a, clampMin), clampMax);
		}
		else if constexpr (!scalar)
		{
			return vmin(vmax(a, clampMin), clampMax);
		}
		assert(false);
		return a; // Failure case; assert false & return the value unclamped
	}

	template<vec2_type_fp vec2>
	vec2 vabs(vec2 v)
	{
		return vec2(
			std::abs(v.e[0]),
			std::abs(v.e[1]));
	}

	template<vec3_type_fp vec3>
	vec3 vabs(vec<3> v)
	{
		return vec3(
			std::abs(v.e[0]),
			std::abs(v.e[1]),
			std::abs(v.e[2]));
	}

	template<vec2_type_integral vec2>
	vec2 vabs(vec2 v)
	{
		return vec2(
			static_cast<uint64_t>(v.e[0]),
			static_cast<uint64_t>(v.e[1]));
	}

	template<vec3_type_fp vec3>
	vec3 vabs(vec3 v)
	{
		return vec3(
			std::abs(v.e[0]),
			std::abs(v.e[1]),
			std::abs(v.e[2]));
	}

	template<vec3_type_integral vec3>
	vec3 vmod(vec3 u, vec3 v)
	{
		return vec3(
			u.e[0] % v.e[0],
			u.e[1] % v.e[1],
			u.e[2] % v.e[2]);
	}

	template<typename vec_elt_type> requires (std::is_integral_v<vec_elt_type>)
		vec<3, vec_elt_type> vmod(vec<3, vec_elt_type> u, vec_elt_type modulus)
	{
		return vec<3, vec_elt_type>(
			u.e[0] % modulus,
			u.e[1] % modulus,
			u.e[2] % modulus);
	}


	template<scalar_maths_type argType>
	argType sgn(argType f)
	{
		const bool sgn = f < argType(0);
		return static_cast<argType>(sgn ? -1 : 1);
	}

	template<vec3_type vec3>
	vec3 vsgn(vec3 v)
	{
		return vec3(
			sgn(v.x()),
			sgn(v.y()),
			sgn(v.z()));
	}

	template<typename num_type> requires(std::is_floating_point_v<num_type>)
		bool eps_equality(num_type x, num_type y)
	{
		return x >= (y - eps) &&
			x <= (y + eps);
	}

	template<vec2_type type_in, vec2_type type_out>
	type_out vec2_cast(type_in vec_in)
	{
		using output_type = typename type_out::vec_type;
		return type_out(static_cast<output_type>(vec_in.e[0]),
			static_cast<output_type>(vec_in.e[1]));
	}

	template<vec3_type type_in, vec3_type type_out>
	type_out vec3_cast(type_in vec_in)
	{
		using output_type = typename type_out::vec_type;
		return type_out(static_cast<output_type>(vec_in.e[0]),
			static_cast<output_type>(vec_in.e[1]),
			static_cast<output_type>(vec_in.e[2]));
	}

	// Higher-level function object/functor; not super fancy but better than std::function (spooky hidden heap allocations) and more readable than c-style function pointers
	template<int dim, maths_type ret>
	struct fn
	{
	private:
		ret(*ptr)(vec<dim>);
	public:
		fn() : ptr(NULL) {};
		fn(decltype(ptr) fnPtr) : ptr(fnPtr) {}
		ret invoke(vec<dim> param) // User-friendly calling interface
		{
			return ptr(param);
		}
	};

	// Extension of [fn] with captured state
	template<int dim, maths_type ret>
	struct fn_closure
	{
	private:
		fn<dim, ret> operation;
	public:
		vec<dim> param_baked;
		fn_closure(vec<dim> _param_baked) : param_baked(_param_baked) {}
		ret invoke()
		{
			return operation(param_baked);
		}
	};
}

#ifdef MATH_DBG
#pragma optimize("", on)
#endif