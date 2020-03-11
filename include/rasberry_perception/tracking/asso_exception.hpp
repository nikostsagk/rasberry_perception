#ifndef ASSO_EXCEPTION_H
#define ASSO_EXCEPTION_H

#include <exception>

class asso_exception : public std::exception {
public:
    const char* what() const noexcept override {
        return "Unknown association algorithm!";
    }
};

class filter_exception : public std::exception {
public:
    const char* what() const noexcept override {
        return "Unknown filter type!";
    }
};

class observ_exception: public std::exception {
public:
  const char* what() const noexcept override {
      return "Unknown observation model!";
  }
};

class reset_exception : public std::exception {
public:
    const char *what() const noexcept override {
        return "Node reset exception raised!";
    }
};

#endif // ASSO_EXCEPTION_H
