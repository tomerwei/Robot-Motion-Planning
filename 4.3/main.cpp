#if defined(_WIN32)
#pragma warning(disable: 4819)
#endif

#include <boost/optional.hpp>

#include <QtGui/QApplication>

#include "MRMPApplication.h"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MRMPApplication w;
  a.setActiveWindow(&w);
  w.show();
  a.setQuitOnLastWindowClosed(true);
  a.connect(qApp, SIGNAL(lastWindowClosed() ), qApp, SLOT( quit()));

  return a.exec();
}
