#ifndef AT_AVISO_H
#define AT_AVISO_H

#include <QDialog>
#include <lib_ATGUI/at_label.h>
#include <lib_ATGUI/at_label.h>
#include "estado_programa.h"

namespace Ui {
class ATAviso;
}

class ATAviso : public QDialog
{
    Q_OBJECT

public:
    explicit ATAviso(QWidget *parent = 0);
    ~ATAviso();
    void modoAviso(int tipo);
    int getTipoAviso();
    ATLabel * getLabelLiberar() const;
private:
    Ui::ATAviso *ui;
    int tipoAviso;
    void setTema();
};

#endif // AT_AVISO_H
