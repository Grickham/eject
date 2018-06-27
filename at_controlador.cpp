#include "at_controlador.h"

ATControlador::ATControlador(QObject *parent) : QObject(parent)
{

}

ATControlador::ATControlador(ATVista* vista,ATModelo* modelo) :
                vista(vista), modelo(modelo),hiloModelo(new QThread(this))
{
    initConexionesVista();
    initConexionesModelo();
    initConexionesVistaAviso();
    secuenciaInicio();
}

/*Metodos privados*/
void ATControlador::initConexionesVista(){
    connect(vista ,SIGNAL(cerrarApi ())         ,this ,SLOT(cerrarAplicacion()) ,Qt::QueuedConnection);
    connect(vista ,SIGNAL(clickedStartStop())   ,this ,SLOT(clickedStartStop()));
//    connect(vista->getBtnParo(),&ATButton::clicked,modelo,&ATModelo::setParo,Qt::QueuedConnection);
//    connect(vista, &ATVista::iniciarSecuenciaInicio,this,&ATControlador::iniciarSoftware);
}

void ATControlador::initConexionesVistaAviso(){
    connect(vista->getVistaAviso()->getLabelLiberar(),&ATLabel::clicked,this,&ATControlador::liberarApi);
}

void ATControlador::liberarApi(int){
    if(vista->getVistaAviso()->getTipoAviso() != AVISO_LIBERAR_PARO)
        return;

    vista->mostrarAviso(false);
    vista->setBlocked(false);
}

void ATControlador::initConexionesModelo(){
    modelo->moveToThread(hiloModelo);

    connect(modelo     ,SIGNAL(finHilo())  ,hiloModelo  ,SLOT(quit()));
    connect(modelo     ,SIGNAL(finHilo())  ,modelo      ,SLOT(deleteLater()));
    connect(hiloModelo ,SIGNAL(finished()) ,hiloModelo  ,SLOT(deleteLater()));

    connect(this ,&ATControlador::salida,modelo,&ATModelo::Salida,Qt::BlockingQueuedConnection);
    connect(this, &ATControlador::senalParo,modelo, &ATModelo::setStopReceived,Qt::BlockingQueuedConnection);
    connect(this,&ATControlador::conectarRobot, modelo, &ATModelo::conectarRobot, Qt::BlockingQueuedConnection);

    connect(modelo,&ATModelo::enParo         ,this,&ATControlador::enParo          ,static_cast<Qt::ConnectionType>(Qt::QueuedConnection | Qt::UniqueConnection));
    connect(modelo,&ATModelo::errorDeConexion,this,&ATControlador::setErrorConexion,static_cast<Qt::ConnectionType> (Qt::BlockingQueuedConnection | Qt::UniqueConnection));
    connect(modelo,&ATModelo::errorMovimiento,this,&ATControlador::salirEjecucion  ,Qt::QueuedConnection);
    connect(modelo,&ATModelo::errorParo      ,this,&ATControlador::setErrorParo    ,Qt::QueuedConnection);
    //lanza el hilo monitor del paro de emergencia
    connect(hiloModelo,&QThread::started,modelo,&ATModelo::lanzarAlarmaMonitor);

    hiloModelo->start(QThread::HighPriority);
}

void ATControlador::initParametros(){
    ejecucionIniciada = EJECUCION_EN_PARO;
    virtualStop = false;
}

void ATControlador::initVistas(){
    mensaje = new ATMensaje(vista);
}

void ATControlador::secuenciaInicio(){
    emit conectarRobot();
}

void ATControlador::setErrorConexion(int numSocket){
    disconnect(modelo,&ATModelo::errorDeConexion,0,0);
    mensajeDescriptivo = "Se perdió la comunicación con el servidor (puerto " + QString::number(numSocket) + "), "
                         "compruebe la conexión.";
    rutaImaBtnAceptarMen = ":/botones/var/assets/botones/aceptar_mensaje.png";

    blockGUI(false);
    mensaje->mensajeAviso(mensajeDescriptivo,rutaImaBtnAceptarMen,QMessageBox::Critical);
    ATUtils::utils_espera_process_events(100);
    std::terminate();
}

void ATControlador::enParo(bool opcion){
    if(opcion){
        virtualStop = true;
        vista->mostrarAviso(false);

        vista->mostrarAviso(opcion,AVISO_PARO);
        QCoreApplication::processEvents();
        do{
            ATUtils::utils_espera_process_events(100);
        }while(future.isRunning());

        ejecucionIniciada = EJECUCION_EN_PARO;
        vista->modoEjecucion(false);

        vista->setBlocked(opcion);
        QCoreApplication::processEvents();
    }

    else{
        while(future.isRunning())
            ATUtils::utils_espera_process_events(100);

        vista->setBlocked(false);
        vista->mostrarAviso(false);
        QCoreApplication::processEvents();
        vista->mostrarAviso(true,AVISO_LIBERAR_PARO);
        vista->setBlocked(true);
        QCoreApplication::processEvents();
    }
}

void ATControlador::salirEjecucion(){
    do{
        ATUtils::utils_espera_process_events(100);
    }while(future.isRunning());
    ///Ventana de esperando a que se libere el robot
    qDebug()<<"\nControlador: esperando a que se libere el paro";
    ejecucionIniciada = EJECUCION_EN_PARO;
    if(!virtualStop){
        vista->modoEjecucion(false);
    }
}

void ATControlador::blockGUI(bool opcion){
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    vista->setBlocked(opcion);
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}

void ATControlador::cerrarAplicacion(){
    emit salida(true);
    do{
         QCoreApplication::processEvents(QEventLoop::AllEvents);
    }while(hiloModelo->isRunning());

    QCoreApplication::quit();
}

void ATControlador::clickedStartStop(){
    //Si no nos encontramos en ejecucion
    if(!ejecucionIniciada){
        virtualStop = false;
        modelo->controladorSetStop(false);
        future    = QtConcurrent::run(modelo, &ATModelo::Eject2);
       // futureIos = QtConcurrent::run(modelo, &ATModelo::monitorIos);
        ejecucionIniciada = EJECUTANDO_RUTINA;
        vista->modoEjecucion(true);
   }

    //detener ejecucion
    else{
        virtualStop = true;
        modelo->controladorSetStop(true);

        blockGUI(true);

        QCoreApplication::processEvents();
        QFuture<void>futureCal;
        futureCal = QtConcurrent::run(this, &ATControlador::setStop);

        do{
            ATUtils::utils_espera_process_events(100);
        }while(futureCal.isRunning() || future.isRunning());

        QCoreApplication::processEvents();
        future.waitForFinished();
        ejecucionIniciada = EJECUCION_EN_PARO;
        blockGUI(false);
        vista->modoEjecucion(false);
        virtualStop = false;
    }
}

void ATControlador::setStop(){
    emit senalParo(true);
}

void ATControlador::setErrorParo(){
    if(mensaje->isActiveWindow())
        return;
    mensajeDescriptivo = "Libere el paro de emergencia, "
                         "posteriormente reinicie la aplicación.";
    rutaImaBtnAceptarMen = ":/botones/var/assets/botones/aceptar_mensaje.png";
    mensaje->mensajeAviso(mensajeDescriptivo,rutaImaBtnAceptarMen,QMessageBox::Critical);
    QCoreApplication::processEvents();
    std::terminate();
}
