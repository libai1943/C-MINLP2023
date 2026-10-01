// Chapter 6: Li et al., IEEE T-IV 8(2), 1512-1522 (2023).
// DOI: 10.1109/TIV.2022.3214777. Constructed dumping-bay example.
// GHA/PMP returns 200 evaluated candidates; solve targeted NLP (14) once.
// The backward steering bound follows the prose/Fig.3 convention and MATLAB.
#include <algorithm>
#include <array>
#include <casadi/casadi.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
extern "C" int mining_reference(double *, int, double *, char *, int);
using casadi::DM;
using casadi::SX;
using Box = std::array<double, 4>;
constexpr int N = 101, Q = 11;
constexpr double wheelbase = 5.73, rear_offset = .435, front_offset = 5.105;
const double radius = std::hypot(9.34 / 4, 3.5 / 2);
const std::array<Box, 5> obstacles{{{60, 84, 38, 44},
                                    {60, 84, 56, 62},
                                    {82, 88, 44, 56},
                                    {30, 43, 5, 15},
                                    {14, 26, 57, 70}}};
bool safe(const Box &b) {
  if (b[0] < radius || b[1] > 100 - radius || b[2] < radius ||
      b[3] > 90 - radius)
    return false;
  for (auto o : obstacles) {
    double dx = std::max({o[0] - b[1], b[0] - o[1], 0.}),
           dy = std::max({o[2] - b[3], b[2] - o[3], 0.});
    if (hypot(dx, dy) < radius + 1e-8)
      return false;
  }
  return true;
}
Box corridor(double x, double y) {
  Box b{x, x, y, y};
  if (!safe(b))
    throw std::runtime_error("Reference disc is not collision-free");
  std::array<double, 4> growth{};
  std::array<bool, 4> done{};
  int remaining = 4;
  while (remaining)
    for (int k = 0; k < 4; ++k)
      if (!done[k]) {
        Box trial = b;
        trial[k] += (k % 2 ? .25 : -.25);
        if (growth[k] + .25 > 5 || !safe(trial)) {
          done[k] = true;
          --remaining;
        } else {
          b = trial;
          growth[k] += .25;
        }
      }
  return b;
}
int main(int argc, char **argv) {
  try {
    std::filesystem::path output = "results";
    std::string linear_solver = "mumps";
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "--output" && i + 1 < argc)
        output = std::filesystem::u8path(argv[++i]);
      else if (a == "--linear-solver" && i + 1 < argc)
        linear_solver = argv[++i];
      else
        throw std::runtime_error(
            "Usage: mining_demo [--output DIR] [--linear-solver mumps]");
    }
    auto start = std::chrono::steady_clock::now();
    std::vector<double> reference(N * 8);
    double info[4]{};
    char message[512]{};
    if (mining_reference(reference.data(), int(reference.size()), info, message,
                         512))
      throw std::runtime_error(message);
    std::cout << "GHA: " << info[2] << " candidates, " << info[3]
              << " expansions, selected cost=" << info[1] << std::endl;
    std::vector<std::array<Box, 2>> boxes(N);
    for (int i = 0; i < N; ++i)
      for (int disk = 0; disk < 2; ++disk) {
        double d = disk ? front_offset : rear_offset;
        boxes[i][disk] =
            corridor(reference[8 * i] + d * cos(reference[8 * i + 2]),
                     reference[8 * i + 1] + d * sin(reference[8 * i + 2]));
      }
    SX z = SX::sym("c", N * Q + 1), tf = z(N * Q), h = tf / (N - 1),
       objective = tf;
    auto q = [&](int i, int j) { return z(i * Q + j); };
    std::vector<double> lower(N * Q + 1, -1e20), upper(N * Q + 1, 1e20),
        guess(N * Q + 1), gl, gu;
    std::vector<SX> expressions;
    auto add = [&](SX e, double l = 0, double u = 0) {
      expressions.push_back(e);
      gl.push_back(l);
      gu.push_back(u);
    };
    lower.back() = .1;
    guess.back() = info[0];
    for (int i = 0; i < N; ++i) {
      int direction = int(reference[i * 8 + 7]);
      double phimax = .49 * (direction < 0 ? .6122 : 1.);
      for (int k = 0; k < 7; ++k)
        guess[i * Q + k] = reference[i * 8 + k];
      for (int k = 0; k < 2; ++k) {
        lower[i * Q + k] = reference[i * 8 + k] - 5;
        upper[i * Q + k] = reference[i * 8 + k] + 5;
      }
      lower[i * Q + 3] = direction > 0 ? 0 : -1;
      upper[i * Q + 3] = direction > 0 ? 2 : 0;
      for (int k = 4; k < 7; ++k) {
        double b = k == 4 ? .2 : k == 5 ? phimax : .14;
        lower[i * Q + k] = -b;
        upper[i * Q + k] = b;
      }
      objective += .025 * (pow(q(i, 4), 2) + pow(q(i, 3) * q(i, 6), 2));
      for (int disk = 0; disk < 2; ++disk) {
        double d = disk ? front_offset : rear_offset;
        int k = 7 + 2 * disk;
        auto b = boxes[i][disk];
        add(q(i, k) - q(i, 0) - d * cos(q(i, 2)));
        add(q(i, k + 1) - q(i, 1) - d * sin(q(i, 2)));
        lower[i * Q + k] = b[0];
        upper[i * Q + k] = b[1];
        lower[i * Q + k + 1] = b[2];
        upper[i * Q + k + 1] = b[3];
        guess[i * Q + k] = reference[i * 8] + d * cos(reference[i * 8 + 2]);
        guess[i * Q + k + 1] =
            reference[i * 8 + 1] + d * sin(reference[i * 8 + 2]);
      }
      if (i < N - 1) {
        add(q(i + 1, 0) - q(i, 0) - h * q(i, 3) * cos(q(i, 2)));
        add(q(i + 1, 1) - q(i, 1) - h * q(i, 3) * sin(q(i, 2)));
        add(q(i + 1, 2) - q(i, 2) - h * q(i, 3) * tan(q(i, 5)) / wheelbase);
        add(q(i + 1, 3) - q(i, 3) - h * q(i, 4));
        add(q(i + 1, 5) - q(i, 5) - h * q(i, 6));
      }
    }
    for (int i : {0, N - 1})
      for (int k = 0; k < 7; ++k) {
        double value = k < 3 ? reference[i * 8 + k] : 0;
        lower[i * Q + k] = upper[i * Q + k] = guess[i * Q + k] = value;
      }
    casadi::Dict options;
    options["print_time"] = false;
    options["ipopt.print_level"] = 0;
    options["ipopt.sb"] = "yes";
    options["ipopt.tol"] = 1e-8;
    options["ipopt.max_iter"] = 3000;
    options["ipopt.max_cpu_time"] = 300.;
    options["ipopt.linear_solver"] = linear_solver;
    if (const char *hsl = std::getenv("HSL_LIBRARY"))
      options["ipopt.hsllib"] = hsl;
    auto solver = casadi::nlpsol(
        "target", "ipopt",
        casadi::SXDict{
            {"x", z}, {"f", objective}, {"g", SX::vertcat(expressions)}},
        options);
    auto result = solver(casadi::DMDict{{"x0", guess},
                                        {"lbx", lower},
                                        {"ubx", upper},
                                        {"lbg", gl},
                                        {"ubg", gu}});
    if (!bool(solver.stats().at("success")))
      throw std::runtime_error(solver.stats().at("return_status").to_string());
    auto solution = result.at("x").nonzeros(),
         constraints = result.at("g").nonzeros();
    double residual = 0, clearance = 1e20;
    for (size_t i = 0; i < solution.size(); ++i)
      residual =
          std::max({residual, lower[i] - solution[i], solution[i] - upper[i]});
    for (size_t i = 0; i < constraints.size(); ++i)
      residual =
          std::max({residual, gl[i] - constraints[i], constraints[i] - gu[i]});
    for (int i = 0; i < N; ++i)
      for (double d : {rear_offset, front_offset}) {
        double x = solution[i * Q] + d * cos(solution[i * Q + 2]),
               y = solution[i * Q + 1] + d * sin(solution[i * Q + 2]);
        clearance = std::min({clearance, x - radius, 100 - x - radius,
                              y - radius, 90 - y - radius});
        for (auto o : obstacles)
          clearance =
              std::min(clearance, hypot(std::max({o[0] - x, x - o[1], 0.}),
                                        std::max({o[2] - y, y - o[3], 0.})) -
                                      radius);
      }
    if (residual > 1e-5 || clearance < -1e-5)
      throw std::runtime_error("Discrete constraint validation failed");
    int shifts = 0, sign = 0;
    double segment = 0, backward_cost = 0;
    std::vector<double> cusp_lengths;
    for (int i = 0; i < N; ++i) {
      double v = solution[i * Q + 3];
      int next = v > 1e-6 ? 1 : v < -1e-6 ? -1 : 0;
      if (next && sign && next != sign) {
        ++shifts;
        cusp_lengths.push_back(segment);
        if (sign < 0)
          backward_cost += pow(std::max(0., segment - 30), 2);
        segment = 0;
      }
      if (next)
        sign = next;
      if (i < N - 1)
        segment += std::abs(v) * solution.back() / (N - 1);
    }
    if (sign < 0)
      backward_cost += pow(std::max(0., segment - 30), 2);
    cusp_lengths.push_back(segment);
    for (size_t k = 1; k + 1 < cusp_lengths.size(); ++k)
      if (cusp_lengths[k] < 5 - 1e-5)
        throw std::runtime_error("Adjacent cusp distance below 5 m");
    std::filesystem::create_directories(output);
    std::ofstream file(output / "trajectory.csv");
    file << std::setprecision(17) << "t,x,y,theta,v,a,phi,omega\n";
    for (int i = 0; i < N; ++i) {
      file << i * solution.back() / (N - 1);
      for (int k = 0; k < 7; ++k)
        file << ',' << solution[i * Q + k];
      file << '\n';
    }
    std::ofstream warm(output / "reference.csv");
    warm << std::setprecision(17) << "t,x,y,theta,v,a,phi,omega,direction\n";
    for (int i = 0; i < N; ++i) {
      warm << i * info[0] / (N - 1);
      for (int k = 0; k < 8; ++k)
        warm << ',' << reference[i * 8 + k];
      warm << '\n';
    }
    std::cout << "Tf=" << solution.back()
              << " s; target cost=" << double(result.at("f")) << "; full cost="
              << double(result.at("f")) + 20 * shifts + backward_cost
              << "; shifts=" << shifts << "; residual=" << residual
              << "; clearance=" << clearance << " m; planning="
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - start)
                     .count()
              << " s\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
