#include "at_vista.h"
#include "at_modelo.h"
#include "at_controlador.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ATVista *      vista = new ATVista();
    ATModelo*      modelo = new ATModelo();
    ATControlador* controlador = new ATControlador(vista,modelo);

    vista -> show();

    return a.exec();
}
