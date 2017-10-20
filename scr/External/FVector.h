//Simple float Vector 3D implementation
#pragma once
#include <cmath>

struct FVector {
	float x;
	float y;
	float z;

	FVector& operator += (const FVector& A) { x += A.x; y += A.y; z += A.z; return *this; };
	FVector& operator -= (const FVector& A) { x -= A.x; y -= A.y; z -= A.z; return *this; };
	FVector& operator *= (float a) { x *= a; y *= a; z *= a; return *this; };
	FVector& operator /= (float a) { x /= a; y /= a; z /= a; return *this; };
};

#define FV_ZERO  FVector{ 0.0f, 0.0f, 0.0f }

#ifdef _IOSTREAM_
#include <iomanip>
inline std::ostream& operator<<(std::ostream& ostr, const FVector& v) {
	ostr << std::fixed << std::setprecision(2);
	ostr << "[ " << std::setw(6) << v.x << ", " << std::setw(6) << v.y << ", " << std::setw(6) << v.z << " ]";
	return ostr;
}
#endif //_IOSTREAM_

inline FVector operator*(const FVector& A, float B){
	return FVector{A.x*B,A.y*B,A.z*B};
}

inline FVector operator*(float A, const FVector& B){
	return FVector{B.x*A,B.y*A,B.z*A};
}

inline FVector operator+(const FVector& A, const FVector& B){
	return FVector{ A.x + B.x, A.y + B.y, A.z + B.z };
}

inline FVector operator-(const FVector& A, const FVector& B){
	return FVector{ A.x - B.x, A.y - B.y, A.z - B.z };
};

inline FVector operator-(const FVector& A) {
	return FVector{ -A.x, -A.y, -A.z };
};

inline float dot(const FVector& A, const FVector& B){
	return A.x*B.x + A.y*B.y + A.z*B.z;
};

inline FVector cross(const FVector& A, const FVector& B){
	return FVector{ A.y*B.z - A.z*B.y, A.z*B.x - A.x*B.z, A.x*B.y - A.y*B.x };
}

inline FVector norm(const FVector& A){
	float length = sqrt(A.x*A.x + A.y*A.y + A.z*A.z);
	if (length == 0.0f) return FV_ZERO;
	return FVector{ A.x / length, A.y / length, A.z / length };
}

inline bool operator==(const FVector& A, const FVector& B){
	if (&A == &B) return true;
	return  A.x == B.x && A.y == B.y && A.z == B.z;
}
inline bool operator!=(const FVector& A, const FVector& B){
	if (&A == &B) return false;
	return  A.x != B.x || A.y != B.y || A.z != B.z;
}
inline bool isOrthogonal(const FVector& A, const FVector& B){
	if (&A == &B) return false;
	return A.x*B.x + A.y*B.y + A.z*B.z == 0.0f;
}

inline bool isColinear(const FVector& A, const FVector& B) {
	if (&A == &B) return true;
	return ( A.y*B.z - A.z*B.y == 0.0f ) && ( A.z*B.x - A.x*B.z == 0.0f ) && ( A.x*B.y - A.y*B.x == 0.0f );
}

inline float Length(const FVector& A){
	if (A == FV_ZERO) return 0.0f;
	return sqrt(A.x*A.x + A.y*A.y + A.z*A.z);
}

inline float LengthSq(const FVector& A){
	if (A == FV_ZERO) return 0.0f;
	return A.x*A.x + A.y*A.y + A.z*A.z;
}