#pragma once

#include <cmath>
#include <array>

struct Vec2
{
    float x{0.0f};
    float y{0.0f};
    
    constexpr Vec2() = default;
    constexpr Vec2(const float x, const float y) : x{x}, y{y} {}
    
    constexpr Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
    constexpr Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
    constexpr Vec2 operator*(const float scalar) const { return {x * scalar, y * scalar}; }
    constexpr Vec2 operator/(const float scalar) const { return {x / scalar, y / scalar}; }
    Vec2 operator-() const { return {-x, -y}; }
    
    constexpr Vec2& operator+=(const Vec2& rhs) { x += rhs.x; y += rhs.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    constexpr Vec2& operator*=(const float scalar) { x *= scalar; y *= scalar; return *this; }
    
    constexpr float Dot(const Vec2& rhs) const { return x * rhs.x + y * rhs.y; }
    constexpr float Cross(const Vec2& rhs) const { return x * rhs.y - y * rhs.x; }
    static Vec2 ZCross(const float angular_vel, const Vec2& r) { return { -angular_vel * r.y, angular_vel * r.x }; }
    
    float Length() const { return sqrt(x * x + y * y); }
    constexpr float LengthSquared() const { return x * x + y * y; }
    
    Vec2 Normalized() const
    {
        const float len = Length();
        return len > 0.0f ? Vec2{x / Length(), y / Length()} : Vec2{0.0f, 0.0f};
    }
    
    // Cartesian Coordinate System (The Y-axis direction is upwards as positive).
    // Counter-Clock-Wise when normal.
    Vec2 Normal() const { return Vec2{-y, x}.Normalized(); }
    
    constexpr Vec2 NormalWithoutNormalized() const { return {-y, x}; }
};

struct Contact
{
    float depth;
    Vec2 normal;
    Vec2 collision_point_a;
    Vec2 collision_point_b;
    size_t a;
    size_t b;
    
    float impulse_sum{0.f};
    float pseudo_impulse_sum{0.f};
    float impulse_sum_tan{0.f};
    
    float mass_normal{0.f};
    float mass_tangent{0.f};
    float bias{0.f};
};

struct BodyHandle
{
    size_t id;
    bool IsValid() const { return id != 0xFFFFFFFF; }
};

template <typename T, size_t Capacity>
struct FixedVector 
{
    std::array<T, Capacity> data{};
    size_t sz = 0;

    void push_back(const T& val) 
    {
        data[sz++] = val;
    }
    
    void insert(size_t index, const T& val)
    {
        for (size_t i = sz; i > index; --i)
        {
            data[i] = data[i - 1];
        }
        data[index] = val;
        ++sz;
    }

    void erase_at(const size_t index) 
    {
        for (size_t i = index; i < sz - 1; ++i) 
        {
            data[i] = data[i + 1];
        }
        --sz;
    }

    T& operator[](size_t i) { return data[i]; }
    const T& operator[](size_t i) const { return data[i]; }

    Vec2& back() { return data[sz - 1]; }
    const Vec2& back() const  { return data[sz - 1]; }
    size_t size() const { return sz; }
    static constexpr size_t capacity() { return Capacity; }
};

using FixedSimplex = FixedVector<Vec2, 3>;
using FixedPolytope = FixedVector<Vec2, 32>;