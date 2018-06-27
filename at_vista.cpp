#include "at_vista.h"
#include "ui_at_vista.h"

ATVista::ATVista(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ATVista)
{
    ui->setupUi(this);
    vistaAviso = new ATAviso(this);
    setTema();
    initConexiones();

}

ATVista::~ATVista()
{
    delete ui;
}

void ATVista::modoEjecucion(bool opcion){
    //si necesito la vista en modo Ejecucion
    if(opcion){
        // Setea la imagen del boton a STOP
        ui->btnStartStop->setIcono(":/botones/var/assets/botones/stop.png");
        ui->btnSalir    ->setEnabled(false);
    }
    //si necesito poner todo a modo ejecucion
    else{
        ui->btnSalir           ->setEnabled(true);
        // Setea la imagen del boton a START
        ui->btnStartStop->setIcono(":/botones/var/assets/botones/st.png");
    }
}

void ATVista::setBlocked(bool opcion){
    this->centralWidget()->setDisabled(opcion);
}

void ATVista::mostrarAviso(bool opcion, int tipoAviso){
    if(opcion){
        vistaAviso->modoAviso(tipoAviso);
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        vistaAviso->show();
    }
    else{
        vistaAviso->hide();
    }

}

/*metodos privados*/

void ATVista::setTema(){
    //ui->btnStart->setId(ESTADO_STOP);
    ui->btnStartStop->setIcono(":/botones/var/assets/botones/st.png");
    ui->btnSalir    ->setIcono(":/botones/var/assets/botones/close.png");
    setStyleSheet("background-color:#FFFFFF;");
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    ui->AtechnikLogo->setImagen(":/logos/var/assets/logos/logo_automatische.png");
}

void ATVista::initConexiones(){
    connect(ui->btnSalir     ,SIGNAL( clicked ()) ,this  ,SLOT(emitCerrarApi()));
    connect(ui->btnStartStop ,SIGNAL( clicked ()) ,this  ,SLOT(clickedBtnStartStop()));
}

void ATVista::emitCerrarApi(){
    emit cerrarApi();
}

void ATVista::clickedBtnStartStop(){
    emit clickedStartStop();
}
