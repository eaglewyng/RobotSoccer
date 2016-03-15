/*
 * Object.cpp
 *
 *  Created on: Jan 30, 2015
 *      Author: lukehsiao
 */

#include "Object.h"

using namespace cv;

Object::Object() {
  this->x_pos = 0;
  this->y_pos = 0;
  this->img_x = 455;
  this->img_y = 240;
  this->old_x_pos = 0;
  this->old_y_pos = 0;
  this->HSVmax = 0;
  this->HSVmin = 0;
}

Object::~Object() {
  // TODO Auto-generated destructor stub
}

// Setters and Getters
void Object::set_x_pos(int x) {
  this->old_x_pos = this->x_pos; // save old position
  this->x_pos = x;
}

int Object::get_x_pos() {
  return this->x_pos;
}

void Object::set_img_x(int x) {
  this->img_x = x;
}

int Object::get_img_x() {
  return this->img_x;
}

void Object::set_img_y(int y) {
  this->img_y = y;
}

int Object::get_img_y() {
  return this->img_y;
}

void Object::set_y_pos(int y) {
  this->old_y_pos = this->y_pos; // save old position
  this->y_pos = y;
}

int Object::get_y_pos() {
  return this->y_pos;
}

int Object::get_old_y() {
  return this->old_y_pos;
}

int Object::get_old_x() {
  return this->old_x_pos;
}

cv::Scalar Object::getHSVmin() {
  return this->HSVmin;
}

void Object::setHSVmin(cv::Scalar min) {
  this->HSVmin = min;
}

cv::Scalar Object::getHSVmax() {
  return this->HSVmax;
}

void Object::setHSVmax(cv::Scalar max) {
  this->HSVmax = max;
}
