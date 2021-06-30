#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <regex>
#include <vector>
#include <cmath>

#include "cimnet/_base_net.h"

class Point {
public:
    Point() = default;

    explicit Point(double x, double y) {
        this->x = x;
        this->y = y;
    }

    Point operator+(const Point &other) const {
        return Point(x + other.x, y + other.y);
    }

    Point operator-(const Point &other) const {
        return Point(x - other.x, y - other.y);
    }

    Point operator+=(const Point &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Point operator-=(const Point &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    double x{};
    double y{};
};

typedef double Distance;
typedef std::vector<Point> Coordinates;
typedef Network<size_t, Point, Distance> SVGPathNetwork;

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


class SVGPainter {
private:
    void _prod_curve_terms() {
        double step = 1.0 / precision;
        for (int i = 1; i <= precision; i++) {
            double t = i * step;
            ct[0].push_back((1 - t) * (1 - t) * (1 - t));
            ct[1].push_back(3 * (1 - t) * (1 - t) * t);
            ct[2].push_back(3 * (1 - t) * t * t);
            ct[3].push_back(t * t * t);
        }
    }

public:

    SVGPainter() {
        _prod_curve_terms();
    }

    explicit SVGPainter(int precision) {
        // If precision is 1, ignore bezier curve.
        this->precision = precision;
        _prod_curve_terms();
    }

    inline Point shorthand_controller() const {
        return Point(2 * last_point.x - last_controller.x,
                     2 * last_point.y - last_controller.y);
    }

    inline void move_to(const Point p1) {
        path.clear();
        path.push_back(last_point = p1);
    }

    inline void close_path() {
        // pass
    }

    inline void line_to(const Point p1, bool relative = true) {
        if (relative) last_point += p1;
        else last_point = p1;
        path.push_back(last_point);
    }

    inline void horizontal_to(double v, bool relative = true) {
        Point point(relative ? last_point.x + v : v, last_point.y);
        line_to(point, false);
    }

    inline void vertical_to(double h, bool relative = true) {
        Point point(last_point.x, relative ? last_point.y + h : h);
        line_to(point, false);
    }

    inline void curve_to(const Point p1, const Point p2, const Point p3, bool relative = true) {
        const Point _p0 = last_point, _p1 = relative ? last_point + p1 : p1,
                _p2 = relative ? last_point + p2 : p2, _p3 = relative ? last_point + p3 : p3;
        last_controller = _p2;
        last_point = _p3;
        for (int i = 0; i < ct[0].size(); i++) {
            path.emplace_back(Point(
                    ct[0][i] * _p0.x + ct[1][i] * _p1.x + ct[2][i] * _p2.x + ct[3][i] * _p3.x,
                    ct[0][i] * _p0.y + ct[1][i] * _p1.y + ct[2][i] * _p2.y + ct[3][i] * _p3.y));
        }
    }

    inline void curve_to(const Point p1, const Point p3, bool relative = true) {
        const Point _p1 = relative ? last_point + p1 : p1,
                _p2 = shorthand_controller(),
                _p3 = relative ? last_point + p3 : p3;
        curve_to(_p1, _p2, _p3, false);
    }

    inline void quadratic_curve_to(const Point p1, const Point p2, bool relative = true) {
        const Point _p1 = relative ? last_point + p1 : p1, _p2 = relative ? last_point + p2 : p2;
        curve_to(last_point, _p1, _p2, false);
    }

    inline void quadratic_curve_to(const Point p2, bool relative = true) {
        const Point _p1 = shorthand_controller(),
                _p2 = relative ? last_point + p2 : p2;
        quadratic_curve_to(_p1, _p2, false);
    }

    inline std::vector<Point> get_path() const {
        return path;
    }


private:
    std::vector<Point> path{};
    std::vector<double> ct[4];    // Curve terms: (1-t)^3, 3(1-t)*t^2, 3(1-t)^2*t, t^3
    Point last_point{};
    Point last_controller{};
    int precision{1};
};

std::vector<Coordinates> get_path_coordinates(const std::string &str) {
    std::vector<Coordinates> list_co;
    const char *s = str.c_str();
    char op;
    double x, y, x1, y1, x2, y2;

    SVGPainter painter(11);

    while ((op = *s++)) {
        while (*s == ' ') s++;
        switch (op) {
            case 'm':
            case 'M':
                x = read_number(s), y = read_number(s);
                painter.move_to(Point(x, y));
                break;
            case 'z':
            case 'Z':
                painter.close_path();
                list_co.push_back(painter.get_path());
                break;
            case 'l':
            case 'L':
                x = read_number(s), y = read_number(s);
                painter.line_to(Point(x, y), !uppercase(op));
                break;
            case 'h':
            case 'H':
                x = read_number(s);
                painter.horizontal_to(x, !uppercase(op));
                break;
            case 'v':
            case 'V':
                y = read_number(s);
                painter.vertical_to(y, !uppercase(op));
                break;
            case 'c':
            case 'C':
                x1 = read_number(s), y1 = read_number(s), x2 = read_number(s),
                y2 = read_number(s), x = read_number(s), y = read_number(s);
                painter.curve_to(Point(x1, y1), Point(x2, y2), Point(x, y), !uppercase(op));
                break;
            case 's':
            case 'S':
                x2 = read_number(s), y2 = read_number(s), x = read_number(s), y = read_number(s);
                painter.curve_to(Point(x2, y2), Point(x, y), !uppercase(op));
                break;
            case 'q':
            case 'Q':
                x1 = read_number(s), y1 = read_number(s), x = read_number(s), y = read_number(s);
                painter.quadratic_curve_to(Point(x1, y1), Point(x, y), !uppercase(op));
                break;
            case 't':
            case 'T':
                x = read_number(s), y = read_number(s);
                painter.quadratic_curve_to(Point(x, y), !uppercase(op));
                break;
            case 'a':
            case 'A':
                die("Arc not implemented.");
                break;
            default:
                break;
        }
    }
    std::cout << "[\n";
    for (auto &cos : list_co) {
        std::cout << "[\n";
        for (auto &co : cos)
            std::cout << "[" << co.x << ", " << co.y << "],\n";
        std::cout << "],\n";
    }
    std::cout << "]\n";
    return list_co;
}

inline double distance(const Point &u, const Point &v) {
    auto x = v.x - u.x;
    auto y = v.y - u.y;
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
        out << n << "," << net[n].x << "," << net[n].y << "\n";
    for (auto &e: net.edges())
        out << e.first << "," << e.second << "\n";
    out.close();
    return 0;
}
