// -*- mode:c++; tab-width:2; indent-tabs-mode:nil; c-basic-offset:2 -*-
/*
 *  LuminanceSource.cpp
 *  zxing
 *
 *  Copyright 2008 ZXing authors All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sstream>
#include <zxing/LuminanceSource.h>
#include <zxing/common/IllegalArgumentException.h>

using zxing::Ref;
using zxing::LuminanceSource;

LuminanceSource::LuminanceSource(int width_, int height_) :width(width_), height(height_) {}

LuminanceSource::~LuminanceSource() {}

bool LuminanceSource::isCropSupported() const {
  return false;
}

Ref<LuminanceSource> LuminanceSource::crop(int, int, int, int) const {
  throw IllegalArgumentException("This luminance source does not support cropping.");
}

bool LuminanceSource::isRotateSupported() const {
  return false;
}

Ref<LuminanceSource> LuminanceSource::rotateCounterClockwise() const {
  throw IllegalArgumentException("This luminance source does not support rotation.");
}
/*
LuminanceSource::operator std::string() const {
  ArrayRef<char> row;
  std::ostringstream oss;
  for (int y = 0; y < getHeight(); y++) {
    row = getRow(y, row);
    for (int x = 0; x < getWidth(); x++) {
      int luminance = row[x] & 0xFF;
      char c;
      if (luminance < 0x40) {
        c = '#';
      } else if (luminance < 0x80) {
        c = '+';
      } else if (luminance < 0xC0) {
        c = '.';
      } else {
        c = ' ';
      }
      oss << c;
    }
    oss << '\n';
  }
  return oss.str();
}
*/
