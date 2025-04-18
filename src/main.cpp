#include "ros/ros.h" 
#include "geometry_msgs/Twist.h" 
#include "turtlesim/Pose.h" 
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <map>
#include <memory>
#include <functional>

class TurtleState {
private:
    double x;
    double y;
    double angle;
    double linvel;
    double angvel;

public:
    TurtleState() : x(0), y(0), angle(0), linvel(0), angvel(0) {}

    void update(const turtlesim::Pose::ConstPtr& msg) {
        x = msg->x;
        y = msg->y;
        angle = msg->theta;
        linvel = msg->linear_velocity;
        angvel = msg->angular_velocity;
    }

    double getX() const { return x; }
    double getY() const { return y; }
    double getAngle() const { return angle; }
    double getLinVel() const { return linvel; }
    double getAngVel() const { return angvel; }
};

class InputHandler {
private:
    struct termios oldattr, newattr;
    int oldflags;

public:
    InputHandler() {
        tcgetattr(STDIN_FILENO, &oldattr);
        newattr = oldattr;
        newattr.c_lflag &= ~(ICANON | ECHO);
        oldflags = fcntl(STDIN_FILENO, F_GETFL, 0);
    }

    ~InputHandler() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
        fcntl(STDIN_FILENO, F_SETFL, oldflags);
    }

    void setNonBlocking(bool nonBlocking) {
        if (nonBlocking) {
            tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
            fcntl(STDIN_FILENO, F_SETFL, oldflags | O_NONBLOCK);
        }
        else {
            tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
            fcntl(STDIN_FILENO, F_SETFL, oldflags);
        }
    }

    int getch() {
        int ch;
        setNonBlocking(false);
        ch = getchar();
        setNonBlocking(true);
        return ch;
    }

    bool kbhit() {
        int ch;
        setNonBlocking(true);
        ch = getchar();

        if (ch != EOF) {
            ungetc(ch, stdin);
            return true;
        }
        return false;
    }

    int readNumber() {
        int value;
        while (!(std::cin >> value)) {
            std::cout << "Invalid input, please enter a number: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        return value;
    }

    double readDouble() {
        double value;
        while (!(std::cin >> value)) {
            std::cout << "Invalid input, please enter a number: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        return value;
    }

    bool readBool() {
        int value;
        while (!(std::cin >> value) || (value != 0 && value != 1)) {
            std::cout << "Invalid input, please enter 0 or 1: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        return value == 1;
    }
};

class Movement {
protected:
    ros::Publisher& publisher;
    TurtleState& state;

public:
    Movement(ros::Publisher& pub, TurtleState& turtleState)
        : publisher(pub), state(turtleState) {
    }

    virtual ~Movement() {}
    virtual void execute() = 0;
};

class LinearMovement : public Movement {
private:
    double distance;
    double speed;
    bool isForward;

public:
    LinearMovement(ros::Publisher& pub, TurtleState& state,
        double dist, double spd, bool forward)
        : Movement(pub, state), distance(dist), speed(spd), isForward(forward) {
    }

    void execute() override {
        geometry_msgs::Twist vel_msg;

        if (isForward) {
            vel_msg.linear.x = abs(speed);
        }
        else {
            vel_msg.linear.x = -abs(speed);
        }

        publisher.publish(vel_msg);
        ROS_INFO("Turtle is moving!");

        double initialtime = ros::Time::now().toSec();
        double traveltime = distance / speed;
        double finaltime = initialtime + traveltime;

        while (ros::Time::now().toSec() <= finaltime) {
            publisher.publish(vel_msg);
            ros::spinOnce();
        }

        vel_msg.linear.x = 0;
        publisher.publish(vel_msg);
        ROS_INFO("Turtle reached its destination!");
    }
};

class RotationalMovement : public Movement {
private:
    double angleRad;
    double angularSpeed;

public:
    RotationalMovement(ros::Publisher& pub, TurtleState& state,
        double angle, double speed = 90 * M_PI / 180)
        : Movement(pub, state), angleRad(angle), angularSpeed(speed) {
    }

    void execute() override {
        if (angleRad == 0) return;

        geometry_msgs::Twist vel_msg;

        if (angleRad > 0) {
            vel_msg.angular.z = angularSpeed;
        }
        else {
            vel_msg.angular.z = -angularSpeed;
        }

        ROS_INFO("Begin alining direction!");

        double initialtime = ros::Time::now().toSec();
        double traveltime = abs(angleRad) / angularSpeed;
        double finaltime = initialtime + traveltime;

        publisher.publish(vel_msg);
        while (ros::Time::now().toSec() <= finaltime) {
            publisher.publish(vel_msg);
            ros::spinOnce();
        }

        vel_msg.angular.z = 0;
        publisher.publish(vel_msg);
        ROS_INFO("Finished aligning!");
    }
};

class SmoothMovement : public Movement {
private:
    double destX;
    double destY;
    double errorThreshold;
    const double KP_DIST = 5;
    const double KP_ANGLE = 5;

public:
    SmoothMovement(ros::Publisher& pub, TurtleState& state,
        double x, double y, double error)
        : Movement(pub, state), destX(x), destY(y), errorThreshold(error) {
    }

    void execute() override {
        geometry_msgs::Twist vel_msg;
        ros::Rate loop_rate(62);

        double move_x = destX - state.getX();
        double move_y = destY - state.getY();
        double move_dist = sqrt((move_x * move_x) + (move_y * move_y));

        while (move_dist > errorThreshold) {
            vel_msg.linear.x = KP_DIST * move_dist;
            if (vel_msg.linear.x > 5) {
                vel_msg.linear.x = 5;
            }

            double dest_angle = std::atan2(move_y, move_x);
            double move_angle = dest_angle - state.getAngle();

            while (abs(move_angle) > M_PI) {
                if (move_angle > 0) {
                    move_angle -= 2 * M_PI;
                }
                else {
                    move_angle += 2 * M_PI;
                }
            }

            vel_msg.angular.z = KP_ANGLE * move_angle;

            publisher.publish(vel_msg);
            ros::spinOnce();

            move_x = destX - state.getX();
            move_y = destY - state.getY();
            move_dist = sqrt((move_x * move_x) + (move_y * move_y));

            loop_rate.sleep();
        }

        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        publisher.publish(vel_msg);

        ROS_INFO("x: %lf, y:%lf", state.getX(), state.getY());
    }
};

class TeleoperationMovement : public Movement {
private:
    InputHandler& input;
    double linearSpeed;
    double angularSpeed;

public:
    TeleoperationMovement(ros::Publisher& pub, TurtleState& state, InputHandler& handler,
        double linSpeed = 2.0, double angSpeed = 2.0)
        : Movement(pub, state), input(handler), linearSpeed(linSpeed), angularSpeed(angSpeed) {
    }

    void teleop(char key) {
        geometry_msgs::Twist vel_msg;

        switch (key) {
        case 'w':
            vel_msg.linear.x = linearSpeed;
            break;
        case 's':
            vel_msg.linear.x = -linearSpeed;
            break;
        case 'a':
            vel_msg.angular.z = angularSpeed;
            break;
        case 'd':
            vel_msg.angular.z = -angularSpeed;
            break;
        case 'q':
            vel_msg.linear.x = linearSpeed;
            vel_msg.angular.z = angularSpeed;
            break;
        case 'e':
            vel_msg.linear.x = linearSpeed;
            vel_msg.angular.z = -angularSpeed;
            break;
        case 'z':
            vel_msg.linear.x = -linearSpeed;
            vel_msg.angular.z = angularSpeed;
            break;
        case 'c':
            vel_msg.linear.x = -linearSpeed;
            vel_msg.angular.z = -angularSpeed;
            break;
        case ' ':
        default:
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            break;
        }

        publisher.publish(vel_msg);
    }

    void execute() override {
        bool running = true;
        char key;

        std::cout << "Keyboard Teleoperation Mode Activated\n";
        std::cout << "Controls:\n";
        std::cout << "w - move forward\n";
        std::cout << "s - move backward\n";
        std::cout << "a - rotate left\n";
        std::cout << "d - rotate right\n";
        std::cout << "q - move forward and turn left\n";
        std::cout << "e - move forward and turn right\n";
        std::cout << "z - move backward and turn left\n";
        std::cout << "c - move backward and turn right\n";
        std::cout << "space - stop\n";
        std::cout << "x - exit teleoperation mode\n";

        input.setNonBlocking(true);
        ros::Rate rate(10);

        while (running && ros::ok()) {
            if (input.kbhit()) {
                key = input.getch();
                if (key == 'x') {
                    running = false;
                    teleop(' ');
                    std::cout << "Exiting teleoperation mode...\n";
                }
                else {
                    teleop(key);
                }
            }

            ros::spinOnce();
            rate.sleep();
        }

        input.setNonBlocking(false);
    }
};

class ComplexMovement : public Movement {
private:
    double distance;
    double speed;
    bool isForward;
    double angle;

public:
    ComplexMovement(ros::Publisher& pub, TurtleState& state,
        double dist, double spd, bool forward, double ang)
        : Movement(pub, state), distance(dist), speed(spd), isForward(forward), angle(ang) {
    }

    void execute() override {
        std::unique_ptr<Movement> rotation = std::make_unique<RotationalMovement>(publisher, state, angle);
        rotation->execute();

        std::unique_ptr<Movement> linear = std::make_unique<LinearMovement>(publisher, state, distance, speed, isForward);
        linear->execute();
    }
};

class Turtle {
private:
    TurtleState state;
    ros::NodeHandle n;
    ros::Publisher velocityPublisher;
    ros::Subscriber poseSubscriber;

public:
    Turtle() {
        velocityPublisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        poseSubscriber = n.subscribe("/turtle1/pose", 10, &Turtle::updatePose, this);
    }

    void updatePose(const turtlesim::Pose::ConstPtr& msg) {
        state.update(msg);
    }

    void executeMovement(std::unique_ptr<Movement> movement) {
        ros::spinOnce();
        movement->execute();
    }

    ros::Publisher& getPublisher() {
        return velocityPublisher;
    }

    TurtleState& getState() {
        return state;
    }
};

bool runProgram(Turtle& turtle, InputHandler& input) {
    std::cout << "Hello, this program has three functions. Please enter 1, 2, or 3.\n";
    std::cout << "1) Specify movement and direction to turtle.\n";
    std::cout << "2) Specify location for turtle to travel to.\n";
    std::cout << "3) Keyboard teleoperation mode.\n";
    std::cout << "Any other number => Exit\n";

    int command = input.readNumber();

    if (command == 1) {
        std::cout << "What is the distance? ";
        double dist = input.readDouble();

        std::cout << "What is the speed to travel at? ";
        double speed = input.readDouble();

        std::cout << "Are you moving forwards? Answer 1 for true or 0 for false. ";
        bool isForward = input.readBool();

        std::cout << "How many degrees to the left would you like to turn? ";
        double angle = input.readDouble() * M_PI / 180;

        std::unique_ptr<Movement> movement = std::make_unique<ComplexMovement>(
            turtle.getPublisher(), turtle.getState(), dist, speed, isForward, angle);
        turtle.executeMovement(std::move(movement));
    }
    else if (command == 2) {
        ros::spinOnce();

        std::cout << "Current pose of turtle is\n";
        std::cout << "x: " << turtle.getState().getX() << "\n";
        std::cout << "y: " << turtle.getState().getY() << "\n";
        std::cout << "angle: " << turtle.getState().getAngle() * 180 / M_PI << "\n";

        std::cout << "Where would you like the turtle to go to?\n";
        std::cout << "x = ";
        double goto_x = input.readDouble();

        std::cout << "y = ";
        double goto_y = input.readDouble();

        std::cout << "error threshold = ";
        double error = input.readDouble();

        std::unique_ptr<Movement> movement = std::make_unique<SmoothMovement>(
            turtle.getPublisher(), turtle.getState(), goto_x, goto_y, error);
        turtle.executeMovement(std::move(movement));
    }
    else if (command == 3) {
        std::unique_ptr<Movement> movement = std::make_unique<TeleoperationMovement>(
            turtle.getPublisher(), turtle.getState(), input);
        turtle.executeMovement(std::move(movement));
    }
    else {
        std::cout << "Exiting program...";
        return true;
    }

    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_runner");
    Turtle turtle;
    InputHandler input;

    while (ros::ok()) {
        if (runProgram(turtle, input)) {
            return 1;
        }
        ros::spinOnce();
    }

    return 0;
}