
#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <array>

typedef struct affine_state {
    double x;
    double y;
    double d;
    double s;
    double score;
    friend std::ostream& operator<<(std::ostream& stream, const affine_state& as) {
        stream << std::fixed << std::setprecision(2) << "[" << as.x << " " << as.y << " " << as.d << " " << as.s << "]";
        return stream;
    }

    affine_state operator+(affine_state rhs) const {
        return {x + rhs.x, y + rhs.y, d + rhs.d, s + rhs.s, score};
    }

    affine_state operator-(affine_state rhs) const {
        return {x - rhs.x, y - rhs.y, d - rhs.d, s - rhs.s, score};
    }

    affine_state operator*(double t) const {
        return {x * t, y * t, d * t, s * t, score};
    }

} affine_state;

//returns the linear interpolation of the affine state of size (k-1).n+1, where
//k is the waypoints.size()
std::vector<affine_state> interpolate_states(const std::vector<affine_state> &waypoints, int n)
{
    size_t k_in = waypoints.size();
    size_t k_out = (k_in-1)*n;
    std::vector<affine_state> output(k_out);
    for(int i = 0; i < k_out; i++) 
    {
        int i_in = i / n;
        double j = (i % n)/(double)n;
        output[i] = waypoints[i_in] + (waypoints[i_in+1] - waypoints[i_in])*j;
    }
    output.push_back(waypoints.back());

    return output;
}


int main(int argc, char* argv[])
{
    //create waypoints
    std::vector<affine_state> waypoints;
    waypoints.push_back({0.0, 0.0, 0.0, 1.0, 0.0});
    waypoints.push_back({100.0, 200.0, 45.0, 1.3, 0.0});
    waypoints.push_back({320.0, 30.0, -32.0, 0.9, 0.0});

    std::cout << "INPUT WAYPOINTS" << std::endl;
    std::cout << "---------------" << std::endl;
    for(auto i : waypoints)
        std::cout << i << std::endl;
    std::cout << std::endl;

    std::vector<affine_state> interpolated = interpolate_states(waypoints, 8);

    std::cout << "OUPUT WAYPOINTS" << std::endl;
    std::cout << "---------------" << std::endl;
    for(auto i : interpolated)
        std::cout << i << std::endl;
    std::cout << std::endl;
    
    return 0;
}