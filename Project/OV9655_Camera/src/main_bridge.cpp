// Bridge between main.c and C++ world

#include <stdint.h>
#include <stdbool.h>
#include <sstream>
#include <zxing/common/FastHybridBinarizer.h>
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>
#include "stm32f4_discovery_lcd.h"

extern "C" void Delay(uint32_t nTime);

using namespace std;
using namespace zxing;

FastHybridBinarizer *binarizer;
Ref<BitMatrix> binary;
bool more = true;
int width_;
int height_;

extern "C" void cppInit(int width, int height) {
  width_ = width;
  height_ = height;
  binarizer = new FastHybridBinarizer(width, height);
  binary = new BitMatrix(width_, height_);
}

extern "C" void cppResetBinarizer(void) {
  binarizer->reset();
}

extern "C" void cppProcessBlockRow(uint8_t *blockRow) {
  binarizer->writeBlockRow(blockRow);
}

extern "C" uint8_t *cppGetResults(void) {
  return binarizer->getResults();
}

vector<Ref<Result> > decode(Ref<BitMatrix> image, DecodeHints hints) {
  Ref<qrcode::QRCodeReader> reader(new qrcode::QRCodeReader);
  return vector<Ref<Result> >(1, reader->decode(image, hints));
}

extern "C" int read_image(void) {
  vector<Ref<Result> > results;
  string cell_result;
  int res = -1;

  try {
    DecodeHints hints(DecodeHints::DEFAULT_HINT);
    hints.setTryHarder(true);
    binary->setBits((int *)binarizer->getResults());
    results = decode(binary, hints);
    res = 0;
  } catch (const ReaderException& e) {
    cell_result = "zxing::ReaderException: " + string(e.what());
    res = -2;
  } catch (const zxing::IllegalArgumentException& e) {
    cell_result = "zxing::IllegalArgumentException: " + string(e.what());
    res = -3;
  } catch (const zxing::Exception& e) {
    cell_result = "zxing::Exception: " + string(e.what());
    res = -4;
  } catch (const std::exception& e) {
    cell_result = "std::exception: " + string(e.what());
    res = -5;
  }

  for (size_t i = 0; i < results.size(); i++) {
    stringstream s;
    char buf[512];
    DCMI_CaptureCmd(DISABLE);
    if (more) {
      s << "Format: "
        << BarcodeFormat::barcodeFormatNames[results[i]->getBarcodeFormat()]
        << endl;
      s.get(buf, sizeof(buf));
      LCD_DisplayStringLine(LINE(2), (unsigned char*)buf);
      for (int j = 0; j < results[i]->getResultPoints()->size(); j++) {
        s.str("");
        s.clear();
        s << "Point[" << j <<  "]: "
          << results[i]->getResultPoints()[j]->getX() << " "
          << results[i]->getResultPoints()[j]->getY() << endl;
        s.get(buf, sizeof(buf));
        LCD_DisplayStringLine(LINE(3 + j), (unsigned char*)buf);
      }
    }
    s.str("");
    s.clear();
    s << results[i]->getText()->getText() << endl;
    s.get(buf, sizeof(buf));
    LCD_DisplayStringLine(LINE(7), (unsigned char*)buf);
    Delay(200);
    LCD_SetDisplayWindow(0, 0, 320, 240);
    LCD_WriteRAM_Prepare();
    DCMI_CaptureCmd(ENABLE);
  }

  return res;
}
