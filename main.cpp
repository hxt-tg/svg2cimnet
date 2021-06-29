#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <regex>
#include <vector>
#include <cmath>

#include "cimnet/_base_net.h"

typedef double Distance;
typedef std::vector<std::pair<double, double>> Coordinates;
typedef Network<size_t, std::pair<double, double>, Distance> SVGPathNetwork;

void die(const char *info) {
    std::cerr << info << "\n";
    exit(-1);
}

inline bool is_numeric(const char &c) {
    return c == '.' || (c <= '9' && c >= '0');
}

inline bool uppercase(const char &c) {
    return c >= 'A' && c <= 'Z';
}

double read_number(const char *&src) {
    const char *p = src;
    std::string buf;
    if (*p == ',' || *p == ' ')
        p++;
    if (*p == '-')
        buf += *p++;
    while (is_numeric(*p))
        buf += *p++;
    src = p;
//    std::cout << "Num: " << buf << "\n";
    return std::stod(buf);
}

std::vector<Coordinates> get_path_coordinates(const std::string &str) {
    std::vector<Coordinates> list_co;
    const char *s = str.c_str();
    char op;
    double x, y;

    double px = 0, py = 0;              // Current painter position

    Coordinates coordinates;
    while ((op = *s++)) {
        while (*s == ' ') s++;
        switch (op) {
            case 'm':
            case 'M':
                x = read_number(s);
                y = read_number(s);
                if (uppercase(op)) px = x, py = y;
                else px += x, py += y;
                coordinates.clear();
                break;
            case 'l':
            case 'L':
                x = read_number(s);
                y = read_number(s);
                if (uppercase(op)) px = x, py = y;
                else px += x, py += y;
                break;
            case 'h':
            case 'H':
                x = read_number(s);
                if (uppercase(op)) px = x;
                else px += x;
                break;
            case 'v':
            case 'V':
                y = read_number(s);
                if (uppercase(op)) py = y;
                else py += y;
                break;
            case 'c':
            case 'C':
                read_number(s);        // x1
                read_number(s);        // y1
                read_number(s);        // x2
                read_number(s);        // y2
                x = read_number(s);
                y = read_number(s);
                if (uppercase(op)) px = x, py = y;
                else px += x, py += y;
                break;
            case 's':
            case 'S':
                read_number(s);        // x2
                read_number(s);        // y2
                x = read_number(s);
                y = read_number(s);
                if (uppercase(op)) px = x, py = y;
                else px += x, py += y;
                break;
            case 'z':
            case 'Z':
                list_co.push_back(coordinates);
                break;
            default:
                break;
        }
        coordinates.push_back(std::make_pair(px, py));
    }
    for (auto &cos : list_co) {
        std::cout << "[\n";
        for (auto &co : cos)
            std::cout << "[" << co.first << ", " << co.second << "],\n";
        std::cout << "],\n";
    }
    return list_co;
}

inline double distance(const std::pair<double, double> &u, const std::pair<double, double> &v) {
    auto x = v.first - u.first;
    auto y = v.second - u.second;
    return pow(x * x + y * y, 0.5);
}

void build_network_from_coordinates(SVGPathNetwork &net, const Coordinates &coordinates) {
    size_t length = net.number_of_nodes();
    net.add_node(length, coordinates[0]);
    for (int i = 1UL; i < coordinates.size(); i++) {
        net.add_node(length + i, coordinates[i]);
        net.add_edge(length + i - 1, length + i, distance(coordinates[i - 1], coordinates[i]));
    }
    net.add_edge(length + coordinates.size() - 1, length,
                 distance(coordinates[coordinates.size() - 1], coordinates[0]));
}

SVGPathNetwork build_network_from_svg(const char *filename) {
    std::ifstream svg(filename);
    if (!svg) die("Cannot open.");

    std::regex space(R"(\n|\s{2,})");
    std::stringstream buf;
    buf << svg.rdbuf();
    std::string str = std::regex_replace(buf.str(), space, "");

    std::smatch m;
    std::regex path("<path d=\"([\\s\\S]+?)\"/>");

    SVGPathNetwork net;
    while (std::regex_search(str, m, path)) {
        std::vector<Coordinates> list_co = get_path_coordinates(m[1]);
        for (auto &coordinates : list_co)
            build_network_from_coordinates(net, coordinates);
        str = m.suffix().str();
    }
    return net;
}

int main() {
    SVGPathNetwork net = build_network_from_svg("../svg_files/CimNet.svg");

    std::ofstream out("../CimNet.graph");
    for (auto &n: net.nodes())
        out << n << "," << net[n].first << "," << net[n].second << "\n";
    for (auto &e: net.edges())
        out << e.first << "," << e.second << "\n";
    out.close();
    return 0;
}
