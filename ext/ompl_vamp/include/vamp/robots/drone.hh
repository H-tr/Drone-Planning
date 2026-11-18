#pragma once

#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots
{
struct Drone
{
    static constexpr char* name = "drone";
    static constexpr std::size_t dimension = 6;
    static constexpr std::size_t n_spheres = 1;
    static constexpr float min_radius = 0.30000001192092896;
    static constexpr float max_radius = 0.30000001192092896;
    static constexpr std::size_t resolution = 64;

    static constexpr std::array<std::string_view, dimension> joint_names = {"base_x_joint", "base_y_joint", "base_z_joint", "base_yaw_joint", "camera_pan", "camera_tilt"};
    static constexpr char* end_effector = "camera";

    using Configuration = FloatVector<dimension>;
    using ConfigurationArray = std::array<FloatT, dimension>;

    struct alignas(FloatVectorAlignment) ConfigurationBuffer
        : std::array<float, Configuration::num_scalars_rounded>
    {
    };

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, n_spheres> x;
        FloatVector<rake, n_spheres> y;
        FloatVector<rake, n_spheres> z;
        FloatVector<rake, n_spheres> r;
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_m{
        20.0, 20.0, 10.0, 6.2831854820251465, 3.1415927410125732, 2.094395160675049
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_a{
        -10.0, -10.0, 0.0, -3.1415927410125732, -1.5707963705062866, -1.5707963705062866
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> d_m{
        0.05000000074505806, 0.05000000074505806, 0.10000000149011612, 0.15915493667125702, 0.31830987334251404, 0.47746482491493225
    };

    static inline void scale_configuration(Configuration& q) noexcept
    {
        q = q * Configuration(s_m) + Configuration(s_a);
    }

    static inline void descale_configuration(Configuration& q) noexcept
    {
        q = (q - Configuration(s_a)) * Configuration(d_m);
    }

    template <std::size_t rake>
    static inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = -10.0 + (q[0] * 20.0);
q[1] = -10.0 + (q[1] * 20.0);
q[2] = 0.0 + (q[2] * 10.0);
q[3] = -3.1415927410125732 + (q[3] * 6.2831854820251465);
q[4] = -1.5707963705062866 + (q[4] * 3.1415927410125732);
q[5] = -1.5707963705062866 + (q[5] * 2.094395160675049);

    }

    template <std::size_t rake>
    static inline void descale_configuration_block(ConfigurationBlock<rake> & q) noexcept
    {
        q[0] = 0.05000000074505806 * (q[0] - -10.0);
q[1] = 0.05000000074505806 * (q[1] - -10.0);
q[2] = 0.10000000149011612 * (q[2] - 0.0);
q[3] = 0.15915493667125702 * (q[3] - -3.1415927410125732);
q[4] = 0.31830987334251404 * (q[4] - -1.5707963705062866);
q[5] = 0.47746482491493225 * (q[5] - -1.5707963705062866);

    }

    inline static auto space_measure() noexcept -> float
    {
        return 165366.8227668515;
    }

    template <std::size_t rake>
    static inline void sphere_fk(const ConfigurationBlock<rake> &x, Spheres<rake> &out) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 4> y;

           // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = x[2];
   y[3] = 0.300000011920929;


        for (auto i = 0U; i < 1; ++i)
        {
            out.x[i] = y[i * 4 + 0];
            out.y[i] = y[i * 4 + 1];
            out.z[i] = y[i * 4 + 2];
            out.r[i] = y[i * 4 + 3];
        }
    }

    using Debug = std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    template <std::size_t rake>
        static inline auto fkcc_debug(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept -> Debug
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 8> y;

           // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = x[2];
   y[3] = 0.300000011920929;
   y[4] = x[0];
   y[5] = x[1];
   y[6] = x[2];
   y[7] = 0.300000011920929;


        Debug output;

        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[0],
                y[1],
                y[2],
                y[3]));
        

        

        return output;
    }

    template <std::size_t rake>
        static inline bool fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 8> y;

           // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = x[2];
   y[3] = 0.300000011920929;
   y[4] = x[0];
   y[5] = x[1];
   y[6] = x[2];
   y[7] = 0.300000011920929;

        





//
// environment vs. robot collisions
//

// base_link
if (sphere_environment_in_collision(environment,
                                    y[4],
                                    y[5],
                                    y[6],
                                    y[7]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        return true;
    }

    template <std::size_t rake>
    static inline bool fkcc_attach(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 9> v;
        std::array<FloatVector<rake, 1>, 20> y;

           y[10] = -0.06 + x[2];
   v[0] = cos(x[3]);
   v[1] = cos(x[4]);
   v[2] = sin(x[3]);
   v[3] = 0. - v[2];
   v[4] = sin(x[4]);
   v[5] = v[0] * v[1] + v[3] * v[4];
   y[19] = cos(x[5]);
   y[11] = v[5] * y[19];
   v[6] = v[2] * v[1] + v[0] * v[4];
   y[12] = v[6] * y[19];
   v[7] = -1. * sin(x[5]);
   y[13] = 0. - v[7];
   v[4] = 0. - v[4];
   v[8] = -1. * -1. * (1. - y[19]) + y[19];
   y[14] = (v[0] * v[4] + v[3] * v[1]) * v[8];
   y[15] = (v[2] * v[4] + v[0] * v[1]) * v[8];
   y[17] = v[5] * v[7];
   y[18] = v[6] * v[7];
   // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[2] = x[2];
   y[3] = 0.300000011920929;
   y[4] = x[0];
   y[5] = x[1];
   y[6] = x[2];
   y[7] = 0.300000011920929;
   y[8] = x[0];
   y[9] = x[1];
   y[16] = 0.;

        





//
// environment vs. robot collisions
//

// base_link
if (sphere_environment_in_collision(environment,
                                    y[4],
                                    y[5],
                                    y[6],
                                    y[7]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        // attaching at camera
        set_attachment_pose(environment, to_isometry(&y[8]));

        //
        // attachment vs. environment collisions
        //
        if (attachment_environment_collision(environment))
        {
            return false;
        }

        //
        // attachment vs. robot collisions
        //

        

        return true;
    }

    static inline auto eefk(const std::array<float, 6> &x) noexcept -> Eigen::Isometry3f
    {
        std::array<float, 9> v;
        std::array<float, 12> y;

           y[2] = -0.06 + x[2];
   v[0] = cos(x[3]);
   v[1] = cos(x[4]);
   v[2] = sin(x[3]);
   v[3] = 0. - v[2];
   v[4] = sin(x[4]);
   v[5] = v[0] * v[1] + v[3] * v[4];
   y[11] = cos(x[5]);
   y[3] = v[5] * y[11];
   v[6] = v[2] * v[1] + v[0] * v[4];
   y[4] = v[6] * y[11];
   v[7] = -1. * sin(x[5]);
   y[5] = 0. - v[7];
   v[4] = 0. - v[4];
   v[8] = -1. * -1. * (1. - y[11]) + y[11];
   y[6] = (v[0] * v[4] + v[3] * v[1]) * v[8];
   y[7] = (v[2] * v[4] + v[0] * v[1]) * v[8];
   y[9] = v[5] * v[7];
   y[10] = v[6] * v[7];
   // dependent variables without operations
   y[0] = x[0];
   y[1] = x[1];
   y[8] = 0.;


        return to_isometry(y.data());
    }
};
}

// NOLINTEND(*-magic-numbers)
