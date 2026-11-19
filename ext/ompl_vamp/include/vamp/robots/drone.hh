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
    static constexpr std::size_t n_spheres = 10;
    static constexpr float min_radius = 0.08780000358819962;
    static constexpr float max_radius = 0.13140399754047394;
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
        std::array<FloatVector<rake, 1>, 3> v;
        std::array<FloatVector<rake, 1>, 40> y;

           v[0] = cos(x[3]);
   v[1] = sin(x[3]);
   v[2] = 0. - v[1];
   y[0] = -0.163933 * v[0] + 0.161097 * v[2] + x[0];
   y[1] = -0.163933 * v[1] + 0.161097 * v[0] + x[1];
   y[4] = 0.155558 * v[0] + 0.160996 * v[2] + x[0];
   y[5] = 0.155558 * v[1] + 0.160996 * v[0] + x[1];
   y[6] = 1e-06 + x[2];
   y[8] = -0.006587 * v[0] + 0.161147 * v[2] + x[0];
   y[9] = -0.006587 * v[1] + 0.161147 * v[0] + x[1];
   y[10] = 1e-05 + x[2];
   y[12] = 0.150841 * v[0] + 0.013792 * v[2] + x[0];
   y[13] = 0.150841 * v[1] + 0.013792 * v[0] + x[1];
   y[14] = -4e-06 + x[2];
   y[16] = 0.169164 * v[0] + -0.146881 * v[2] + x[0];
   y[17] = 0.169164 * v[1] + -0.146881 * v[0] + x[1];
   y[20] = 0.018668 * v[0] + -0.15564 * v[2] + x[0];
   y[21] = 0.018668 * v[1] + -0.15564 * v[0] + x[1];
   y[22] = -1e-06 + x[2];
   y[24] = -0.149995 * v[0] + -0.163411 * v[2] + x[0];
   y[25] = -0.149995 * v[1] + -0.163411 * v[0] + x[1];
   y[28] = -0.169972 * v[0] + -0.005049 * v[2] + x[0];
   y[29] = -0.169972 * v[1] + -0.005049 * v[0] + x[1];
   y[30] = 1e-06 + x[2];
   y[32] = -0.031583 * v[0] + 0.012311 * v[2] + x[0];
   y[33] = -0.031583 * v[1] + 0.012311 * v[0] + x[1];
   y[34] = 5e-06 + x[2];
   y[38] = -0.06 + x[2];
   // dependent variables without operations
   y[2] = x[2];
   y[3] = 0.116599999368191;
   y[7] = 0.121711999177933;
   y[11] = 0.129785999655724;
   y[15] = 0.12432699650526;
   y[18] = x[2];
   y[19] = 0.123264998197556;
   y[23] = 0.131403997540474;
   y[26] = x[2];
   y[27] = 0.124007001519203;
   y[31] = 0.12466699630022;
   y[35] = 0.130707994103432;
   y[36] = x[0];
   y[37] = x[1];
   y[39] = 0.0878000035881996;


        for (auto i = 0U; i < 10; ++i)
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
        std::array<FloatVector<rake, 1>, 3> v;
        std::array<FloatVector<rake, 1>, 44> y;

           v[0] = cos(x[3]);
   v[1] = sin(x[3]);
   v[2] = 0. - v[1];
   y[0] = -0.163933 * v[0] + 0.161097 * v[2] + x[0];
   y[1] = -0.163933 * v[1] + 0.161097 * v[0] + x[1];
   y[4] = 0.155558 * v[0] + 0.160996 * v[2] + x[0];
   y[5] = 0.155558 * v[1] + 0.160996 * v[0] + x[1];
   y[6] = 1e-06 + x[2];
   y[8] = -0.006587 * v[0] + 0.161147 * v[2] + x[0];
   y[9] = -0.006587 * v[1] + 0.161147 * v[0] + x[1];
   y[10] = 1e-05 + x[2];
   y[12] = 0.150841 * v[0] + 0.013792 * v[2] + x[0];
   y[13] = 0.150841 * v[1] + 0.013792 * v[0] + x[1];
   y[14] = -4e-06 + x[2];
   y[16] = 0.169164 * v[0] + -0.146881 * v[2] + x[0];
   y[17] = 0.169164 * v[1] + -0.146881 * v[0] + x[1];
   y[20] = 0.018668 * v[0] + -0.15564 * v[2] + x[0];
   y[21] = 0.018668 * v[1] + -0.15564 * v[0] + x[1];
   y[22] = -1e-06 + x[2];
   y[24] = -0.149995 * v[0] + -0.163411 * v[2] + x[0];
   y[25] = -0.149995 * v[1] + -0.163411 * v[0] + x[1];
   y[28] = -0.169972 * v[0] + -0.005049 * v[2] + x[0];
   y[29] = -0.169972 * v[1] + -0.005049 * v[0] + x[1];
   y[30] = 1e-06 + x[2];
   y[32] = -0.031583 * v[0] + 0.012311 * v[2] + x[0];
   y[33] = -0.031583 * v[1] + 0.012311 * v[0] + x[1];
   y[34] = 5e-06 + x[2];
   y[38] = -0.06 + x[2];
   y[40] = 0.00100651977118105 * v[0] + 0.000457263406133279 * v[2] + x[0];
   y[41] = 0.00100651977118105 * v[1] + 0.000457263406133279 * v[0] + x[1];
   // dependent variables without operations
   y[2] = x[2];
   y[3] = 0.116599999368191;
   y[7] = 0.121711999177933;
   y[11] = 0.129785999655724;
   y[15] = 0.12432699650526;
   y[18] = x[2];
   y[19] = 0.123264998197556;
   y[23] = 0.131403997540474;
   y[26] = x[2];
   y[27] = 0.124007001519203;
   y[31] = 0.12466699630022;
   y[35] = 0.130707994103432;
   y[36] = x[0];
   y[37] = x[1];
   y[39] = 0.0878000035881996;
   y[42] = x[2];
   y[43] = 0.346839368343353;


        Debug output;

        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[0],
                y[1],
                y[2],
                y[3]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[4],
                y[5],
                y[6],
                y[7]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[8],
                y[9],
                y[10],
                y[11]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[12],
                y[13],
                y[14],
                y[15]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[16],
                y[17],
                y[18],
                y[19]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[20],
                y[21],
                y[22],
                y[23]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[24],
                y[25],
                y[26],
                y[27]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[28],
                y[29],
                y[30],
                y[31]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[32],
                y[33],
                y[34],
                y[35]));
        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[36],
                y[37],
                y[38],
                y[39]));
        

        

        return output;
    }

    template <std::size_t rake>
        static inline bool fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 3> v;
        std::array<FloatVector<rake, 1>, 44> y;

           v[0] = cos(x[3]);
   v[1] = sin(x[3]);
   v[2] = 0. - v[1];
   y[0] = -0.163933 * v[0] + 0.161097 * v[2] + x[0];
   y[1] = -0.163933 * v[1] + 0.161097 * v[0] + x[1];
   y[4] = 0.155558 * v[0] + 0.160996 * v[2] + x[0];
   y[5] = 0.155558 * v[1] + 0.160996 * v[0] + x[1];
   y[6] = 1e-06 + x[2];
   y[8] = -0.006587 * v[0] + 0.161147 * v[2] + x[0];
   y[9] = -0.006587 * v[1] + 0.161147 * v[0] + x[1];
   y[10] = 1e-05 + x[2];
   y[12] = 0.150841 * v[0] + 0.013792 * v[2] + x[0];
   y[13] = 0.150841 * v[1] + 0.013792 * v[0] + x[1];
   y[14] = -4e-06 + x[2];
   y[16] = 0.169164 * v[0] + -0.146881 * v[2] + x[0];
   y[17] = 0.169164 * v[1] + -0.146881 * v[0] + x[1];
   y[20] = 0.018668 * v[0] + -0.15564 * v[2] + x[0];
   y[21] = 0.018668 * v[1] + -0.15564 * v[0] + x[1];
   y[22] = -1e-06 + x[2];
   y[24] = -0.149995 * v[0] + -0.163411 * v[2] + x[0];
   y[25] = -0.149995 * v[1] + -0.163411 * v[0] + x[1];
   y[28] = -0.169972 * v[0] + -0.005049 * v[2] + x[0];
   y[29] = -0.169972 * v[1] + -0.005049 * v[0] + x[1];
   y[30] = 1e-06 + x[2];
   y[32] = -0.031583 * v[0] + 0.012311 * v[2] + x[0];
   y[33] = -0.031583 * v[1] + 0.012311 * v[0] + x[1];
   y[34] = 5e-06 + x[2];
   y[38] = -0.06 + x[2];
   y[40] = 0.00100651977118105 * v[0] + 0.000457263406133279 * v[2] + x[0];
   y[41] = 0.00100651977118105 * v[1] + 0.000457263406133279 * v[0] + x[1];
   // dependent variables without operations
   y[2] = x[2];
   y[3] = 0.116599999368191;
   y[7] = 0.121711999177933;
   y[11] = 0.129785999655724;
   y[15] = 0.12432699650526;
   y[18] = x[2];
   y[19] = 0.123264998197556;
   y[23] = 0.131403997540474;
   y[26] = x[2];
   y[27] = 0.124007001519203;
   y[31] = 0.12466699630022;
   y[35] = 0.130707994103432;
   y[36] = x[0];
   y[37] = x[1];
   y[39] = 0.0878000035881996;
   y[42] = x[2];
   y[43] = 0.346839368343353;

        





//
// environment vs. robot collisions
//

// base_link
if (sphere_environment_in_collision(environment,
                                    y[40],
                                    y[41],
                                    y[42],
                                    y[43]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[4],
                                        y[5],
                                        y[6],
                                        y[7]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[8],
                                        y[9],
                                        y[10],
                                        y[11]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[12],
                                        y[13],
                                        y[14],
                                        y[15]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[16],
                                        y[17],
                                        y[18],
                                        y[19]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[20],
                                        y[21],
                                        y[22],
                                        y[23]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[24],
                                        y[25],
                                        y[26],
                                        y[27]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[28],
                                        y[29],
                                        y[30],
                                        y[31]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[32],
                                        y[33],
                                        y[34],
                                        y[35]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[36],
                                        y[37],
                                        y[38],
                                        y[39]))
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
        std::array<FloatVector<rake, 1>, 56> y;

           v[0] = cos(x[3]);
   v[1] = sin(x[3]);
   v[2] = 0. - v[1];
   y[0] = -0.163933 * v[0] + 0.161097 * v[2] + x[0];
   y[1] = -0.163933 * v[1] + 0.161097 * v[0] + x[1];
   y[4] = 0.155558 * v[0] + 0.160996 * v[2] + x[0];
   y[5] = 0.155558 * v[1] + 0.160996 * v[0] + x[1];
   y[6] = 1e-06 + x[2];
   y[8] = -0.006587 * v[0] + 0.161147 * v[2] + x[0];
   y[9] = -0.006587 * v[1] + 0.161147 * v[0] + x[1];
   y[10] = 1e-05 + x[2];
   y[12] = 0.150841 * v[0] + 0.013792 * v[2] + x[0];
   y[13] = 0.150841 * v[1] + 0.013792 * v[0] + x[1];
   y[14] = -4e-06 + x[2];
   y[16] = 0.169164 * v[0] + -0.146881 * v[2] + x[0];
   y[17] = 0.169164 * v[1] + -0.146881 * v[0] + x[1];
   y[20] = 0.018668 * v[0] + -0.15564 * v[2] + x[0];
   y[21] = 0.018668 * v[1] + -0.15564 * v[0] + x[1];
   y[22] = -1e-06 + x[2];
   y[24] = -0.149995 * v[0] + -0.163411 * v[2] + x[0];
   y[25] = -0.149995 * v[1] + -0.163411 * v[0] + x[1];
   y[28] = -0.169972 * v[0] + -0.005049 * v[2] + x[0];
   y[29] = -0.169972 * v[1] + -0.005049 * v[0] + x[1];
   y[30] = 1e-06 + x[2];
   y[32] = -0.031583 * v[0] + 0.012311 * v[2] + x[0];
   y[33] = -0.031583 * v[1] + 0.012311 * v[0] + x[1];
   y[34] = 5e-06 + x[2];
   y[38] = -0.06 + x[2];
   y[40] = 0.00100651977118105 * v[0] + 0.000457263406133279 * v[2] + x[0];
   y[41] = 0.00100651977118105 * v[1] + 0.000457263406133279 * v[0] + x[1];
   y[46] = -0.06 + x[2];
   v[3] = cos(x[4]);
   v[4] = sin(x[4]);
   v[5] = v[0] * v[3] + v[2] * v[4];
   y[55] = cos(x[5]);
   y[47] = v[5] * y[55];
   v[6] = v[1] * v[3] + v[0] * v[4];
   y[48] = v[6] * y[55];
   v[7] = -1. * sin(x[5]);
   y[49] = 0. - v[7];
   v[4] = 0. - v[4];
   v[8] = -1. * -1. * (1. - y[55]) + y[55];
   y[50] = (v[0] * v[4] + v[2] * v[3]) * v[8];
   y[51] = (v[1] * v[4] + v[0] * v[3]) * v[8];
   y[53] = v[5] * v[7];
   y[54] = v[6] * v[7];
   // dependent variables without operations
   y[2] = x[2];
   y[3] = 0.116599999368191;
   y[7] = 0.121711999177933;
   y[11] = 0.129785999655724;
   y[15] = 0.12432699650526;
   y[18] = x[2];
   y[19] = 0.123264998197556;
   y[23] = 0.131403997540474;
   y[26] = x[2];
   y[27] = 0.124007001519203;
   y[31] = 0.12466699630022;
   y[35] = 0.130707994103432;
   y[36] = x[0];
   y[37] = x[1];
   y[39] = 0.0878000035881996;
   y[42] = x[2];
   y[43] = 0.346839368343353;
   y[44] = x[0];
   y[45] = x[1];
   y[52] = 0.;

        





//
// environment vs. robot collisions
//

// base_link
if (sphere_environment_in_collision(environment,
                                    y[40],
                                    y[41],
                                    y[42],
                                    y[43]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[4],
                                        y[5],
                                        y[6],
                                        y[7]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[8],
                                        y[9],
                                        y[10],
                                        y[11]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[12],
                                        y[13],
                                        y[14],
                                        y[15]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[16],
                                        y[17],
                                        y[18],
                                        y[19]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[20],
                                        y[21],
                                        y[22],
                                        y[23]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[24],
                                        y[25],
                                        y[26],
                                        y[27]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[28],
                                        y[29],
                                        y[30],
                                        y[31]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[32],
                                        y[33],
                                        y[34],
                                        y[35]))
    {
        return false;
    }
    
    
    if (sphere_environment_in_collision(environment,
                                        y[36],
                                        y[37],
                                        y[38],
                                        y[39]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        // attaching at camera
        set_attachment_pose(environment, to_isometry(&y[44]));

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
