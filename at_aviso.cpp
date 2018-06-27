#include "at_aviso.h"
#include "ui_at_aviso.h"

ATAviso::ATAviso(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ATAviso)
{
    ui->setupUi(this);
    setTema();
}

ATAviso::~ATAviso()
{
    delete ui;
}
///*** Metodos Publicos ***///

void ATAviso::modoAviso(int tipo){
    tipoAviso = tipo;
    switch(tipo){
    case AVISO_REANUDANDO_ROBOT:
//        setAttribute(Qt::WA_TranslucentBackground,false);
        ui->labelAviso->cambiarAnimacion(":/otros/var/assets/otros/box.gif",200,200);
//        setStyleSheet("background-color:white;");
        ui->labelLogo->hide();
        ui->labelAviso->setMinimumSize(QSize(200,200));
        ui->labelAviso->setMaximumSize(QSize(200,200));
        this->setMinimumSize(200,250);
        this->setMaximumSize(200,250);
        this->setStyleSheet("QDialog {border: 5px solid rgb(3, 117, 216);border-radius: 5px;}");
        break;
    case AVISO_PARO:
        setAttribute(Qt::WA_TranslucentBackground,false);
        ui->labelAviso->cambiarAnimacion(":/otros/var/assets/otros/paro.gif",400,400);
        ui->labelLogo->hide();
        ui->labelAviso->setMinimumSize(QSize(450,450));
        ui->labelAviso->setMaximumSize(QSize(450,450));
        this->setMinimumSize(450,450);
        this->setMaximumSize(450,450);
        this->setStyleSheet("QDialog {border: 5px solid rgb(255, 0, 0);border-radius: 5px;}");
        setStyleSheet("background:transparent;");
        break;
    case AVISO_MOV_ROBOT:
        ui->labelAviso->cambiarAnimacion(":/otros/var/assets/otros/box_mov.gif",200,200);
        this->setStyleSheet("QDialog {border: 5px solid rgb(3, 117, 216);border-radius: 5px;}");
        ui->labelLogo->hide();
        ui->labelAviso->setMinimumSize(QSize(200,200));
        ui->labelAviso->setMaximumSize(QSize(200,200));
        this->setMinimumSize(200,250);
        this->setMaximumSize(200,250);
        break;
    case AVISO_CARGANDO:
        setAttribute(Qt::WA_TranslucentBackground);
        ui->labelLogo->show();
        this->setMinimumSize(QSize(406,91));
        this->setMaximumSize(QSize(406,91));
        ui->labelAviso->cambiarAnimacion(":/otros/var/assets/otros/loading_bar.gif",406,10);
        setStyleSheet("background:transparent;");
        ui->labelAviso->setMinimumSize(QSize(406,10));
        ui->labelAviso->setMaximumSize(QSize(406,10));
        break;
    case AVISO_LIBERAR_PARO:
        setAttribute(Qt::WA_TranslucentBackground);
        ui->labelLogo->hide();
        this->setMinimumSize(QSize(500,500));
        this->setMaximumSize(QSize(500,500));
        ui->labelAviso->cambiarAnimacion(":/otros/var/assets/otros/lib_interfaz.gif",400,400);
        setStyleSheet("background:transparent;");
        ui->labelAviso->setMinimumSize(QSize(500,500));
        ui->labelAviso->setMaximumSize(QSize(500,500));
        break;
    }
}

int ATAviso::getTipoAviso(){
    return tipoAviso;
}

ATLabel * ATAviso::getLabelLiberar() const{
    return ui->labelAviso;
}

///*** Metodos Privados ***///

void ATAviso::setTema(){
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_TranslucentBackground);
    this->setStyleSheet("QDialog {background:transparent;border: 5px solid rgb(3, 117, 216);border-radius: 5px;}");
    ui->labelAviso->setAnimacion(":/otros/var/assets/otros/box.gif",200,300);
    ui->labelLogo->setImagen(":/otros/var/assets/otros/logo_at.png");
    ui->labelLogo->setMinimumSize(65,62);
    ui->labelLogo->setMaximumSize(65,62);
    ui->labelLogo->hide();
    setStyleSheet("background:transparent;");
}
