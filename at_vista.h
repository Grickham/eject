#ifndef AT_VISTA_H
#define AT_VISTA_H

#include <QMainWindow>
#include <lib_ATGUI/at_button.h>
#include <lib_ATGUI/at_label.h>

#include "at_aviso.h"
#include "estado_programa.h"

namespace Ui {
class ATVista;
}

class ATVista : public QMainWindow
{
    Q_OBJECT

public:
    explicit ATVista(QWidget *parent = 0);
    ~ATVista();
    void modoEjecucion(bool opcion);
    void setBlocked(bool opcion);
    void mostrarAviso(bool opcion, int tipoAviso = AVISO_REANUDANDO_ROBOT);
    ATAviso* getVistaAviso(){return vistaAviso;}
signals:
    void cerrarApi();
    void clickedStartStop();
private:
    Ui::ATVista *ui;

    ATAviso * vistaAviso;
    void setTema();
    void initConexiones();
private slots:
    void emitCerrarApi();
    void clickedBtnStartStop();
};

#endif // AT_VISTA_H
