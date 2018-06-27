#include "at_modelo.h"

#define PIN_FRENO 12
#define PIN_LLAVE 1
#define PIN_PARO 0
#define TIEMPO_LECTURA_MS 1000
#define SEGURO_ON 5
#define SEGURO_OFF 6

//Respuestas del servidor al monitor de alarma
#define PARO_FISICO 3
#define PARO_FISICO_LIBERADO 4
#define NUM_MAX_REINTENTOS_INICIO 3
//respuestas de la reanudacion de monitor alarma
#define REINTENTAR 1
#define SALIR -1
#define COMANDO_COMPLETADO 255
bool sen1;
bool sen2;
bool sen3;
int z=775;
int x=0;
int y=0;

ATModelo::ATModelo(QObject *parent) : QObject(parent),
    stopReceived(false),stopReceivedIos(false),sensor1(false),sensor2(false),sensor3(false)
{
    salida = false;
    estadoParoInicio = false;
    socketPrincipalConectado = false;
    posicionSegura.x = 0;
    posicionSegura.y = 0;
    posicionSegura.z = 775;
}

void ATModelo::Salida(bool opcion){
    if(opcion){
        aPosicionEspera(robot);
    }
    mutexSalida.lock();
    salida = true;
    mutexSalida.unlock();

    robot->salidasActuadorPLC(PIN_FRENO,false);
    ATUtils::utils_espera_process_events(500);
    robot->sync(false);
    robot->switchON(false);
    robot->plcReady(false);
    robot->desconectar();

    while(!robotParoFinalizado)
        ATUtils::utils_espera_process_events(100);
    delete robot;
    emit finHilo();
}

void ATModelo::controladorSetStop(bool estado){
    //QMutex mutexControladorParo;
    //mutexControladorParo.lock();
    mutexSenalParo.lock();
    stopReceived = estado;
    mutexSenalParo.unlock();
    //mutexControladorParo.unlock();
    mutexParoIos.lock();
    stopReceivedIos = estado;
    mutexParoIos.unlock();
}

void ATModelo::controladorSetStopIos(bool estado){
    mutexParoIos.lock();
    stopReceivedIos = estado;
    mutexParoIos.unlock();
}

void ATModelo::inicializarRobot(){
    int repeticiones = 0;
    int valor        = 0;

    robot->setPinParo(0);
    robot->switchON(true);
    ATUtils::utils_espera_process_events(500);
    robot->salidasActuadorPLC(PIN_FRENO,true);

    secuenciaInicial:

    // Plc ready ON
    valor = robot->plcReady(true);
    switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
    case SALIR:
        errorDeConexion(PUERTO_CALIBRACION);
        return;
    case REINTENTAR:
        goto secuenciaInicial;
    }

    // Mov sin cinematica
    valor = robot->movSinCinematica();
    switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
    case SALIR:
        errorDeConexion(PUERTO_CALIBRACION);
        return;
    case REINTENTAR:
        goto secuenciaInicial;
    }

    // HOMMING
    valor = robot->homming();
    switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
    case SALIR:
        errorComunicacion(PUERTO_CALIBRACION);
        return;
    case REINTENTAR:
        goto secuenciaInicial;
    }

    // Sync ON
    if(repeticiones < NUM_MAX_REINTENTOS_INICIO){
        valor = robot->sync(SYNC_ON);
        switch (comprobarComando(robot,valor,&repeticiones,PUERTO_CALIBRACION)) {
        case SALIR:
            errorComunicacion(PUERTO_CALIBRACION);
            return;
        }
    }

    if(comprobarErrores(robot,&repeticiones))
        goto secuenciaInicial;

    if(repeticiones == NUM_MAX_REINTENTOS_INICIO){
        errorComunicacion(PUERTO_CALIBRACION);
        return;
    }

}

void ATModelo::errorComunicacion(int numSocket){
    qDebug()<<"Error de conexion";
}

bool ATModelo::getStopReceived(){
    bool aux;
    mutexSenalParo.lock();
    aux = stopReceived;
    mutexSenalParo.unlock();
    return aux;
}

bool ATModelo::getStopReceivedIos(){
    bool aux;
    mutexParoIos.lock();
    aux = stopReceivedIos;
    mutexParoIos.unlock();
    return aux;
}
//Controlador cpp clicked start stop    <----- para cambiar la funcion que se ejecuta
//monitor ios
//0,0,775





void ATModelo::Eject(){ // cambiar coordenadas
    int prioridad=1;
    int random=1;
    qDebug()<<"Iniciando Ejecucion";
    robotEjecucion = new ATRobot(PUERTO_EJECUCION);
    //Ir a la posicion de espera
    robotEjecucion->salidasActuadorPLC(0,false);

    bool banderaEstadoEjec=true;




    bool inicio =true;
    int vel=300;
    int acel=1;
    int zp=920;
    int zso = 890;
    int primerpaso=0;
    int memoria;

    prioridad =0;
    int c1=0;
    int c2=0;
    int c3=0;
    random =0;
    int ze=840; // CUIDADO CUIDADO CUIDADO

    while (prioridad==0){
        sen1=robotEjecucion->statusPinEntrada(5);
        sen2=robotEjecucion->statusPinEntrada(4);
        sen3=robotEjecucion->statusPinEntrada(3);
        if(sen1==true){
            c1+=1;
        }
        else if(sen2==true){
            c2+=1;
        }
        else if(sen3==true){
            c3+=1;
        }
        if(c1>0){
            prioridad=1;
        }
        else if(c2>0){
            prioridad=2;
        }
        else if(c3>0){
            prioridad=3;
        }
        if(getStopReceived()){
            detenerEjecucion();
            return;
        }

    }


    switch(prioridad){
    case 1:
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,19,775,vel) != COMANDO_EJECUTADO){
            qDebug("aquic1");
           break;

        }
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,19,zp,vel) != COMANDO_EJECUTADO){
          break;
        }
        robotEjecucion->salidasActuadorPLC(0,true);
        c1-=1;
        if(getStopReceived()){
            if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                return;
            }
            return;
        }

        primerpaso=1;
        break;
    case 2:
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-53,775,vel) != COMANDO_EJECUTADO){
            break;

        }
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-53,zp,vel) != COMANDO_EJECUTADO){
            break;
        }
        c2-=1;
        robotEjecucion->salidasActuadorPLC(0,true);
        if(getStopReceived()){
            if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                return;
            }
            return;
        }
        primerpaso=1;
        break;
    case 3:
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-118,775,vel) != COMANDO_EJECUTADO){
            break;
        }
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-118,zp,vel) != COMANDO_EJECUTADO){
            break;
        }
        c3-=1;
        robotEjecucion->salidasActuadorPLC(0,true);
        if(getStopReceived()){
            if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                return;
            }
            return;
        }
        primerpaso=1;
        break;
    }
qDebug("antes while");
    while (inicio){

        random = 1+rand()%3;
        banderaEstadoEjec=true;
        do{
            if (primerpaso==1){
                break;
            }
            sen1=robotEjecucion->statusPinEntrada(5);
            sen2=robotEjecucion->statusPinEntrada(4);
            sen3=robotEjecucion->statusPinEntrada(3);

            if(sen1==true){
                c1+=1;
            }
            else if(sen2==true){
                c2+=1;
            }
            else if(sen3==true){
                c3+=1;
            }
            if(c1>0){
                prioridad=1;
                break;
            }
            else if(c2>0){
                prioridad=2;
                break;
            }
            else if(c3>0){
                prioridad=3;
                break;
            }



            if(getStopReceived()){
                if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                    return;
                }
                return;
            }
            if(sen1==false && sen2==false && sen3==false){
                prioridad=0;

            }


        }while(prioridad==0);


        while (primerpaso==1){
            if (primerpaso==1){
                memoria=prioridad;
                prioridad=0;
                primerpaso=0;
                if(c1>=1){
                    c1-=1;
                }
                else if(c2>=1){
                    c2-=1;
                }
                else if(c3>=1){
                    c3-=1;
                }

            }
        }

        switch(prioridad){
        case 1:
            if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-121,19,zp,vel,40,acel,ze) != COMANDO_EJECUTADO){
 qDebug("aquic1");
                break;
            }
            c1-=1;
            robotEjecucion->salidasActuadorPLC(0,true);
            if(getStopReceived()){
                if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                    return;
                }
                return;
            }

            break;
        case 2:if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-121,-53,zp,vel,40,acel,ze) != COMANDO_EJECUTADO){
                //detenerEjecucion();
                break;
            }
            c2-=1;
            robotEjecucion->salidasActuadorPLC(0,true);
            if(getStopReceived()){
                if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                    return;
                }
                return;
            }

            break;
        case 3: if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-121,-118,zp,vel,40,acel,ze) != COMANDO_EJECUTADO){

                break;
            }
            c3-=1;
            robotEjecucion->salidasActuadorPLC(0,true);
            if(getStopReceived()){
                if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                    return;
                }
                return;
            }
        case 0:
            prioridad=memoria;
            break;
        }
        while (banderaEstadoEjec==true){
            if (prioridad==1 && random==1){
               qDebug("aquir1");
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,245,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    // banderaEstadoEjec=false; poner bandera de estado para el while oh no o.O
                    break;
                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                }
                robotEjecucion->salidasActuadorPLC(0,false);

            }

            else if (prioridad==1 && random==2){
                qDebug("aquir2");
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,306,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==1 && random==3){
               qDebug("aquir3");
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,371,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }


            if (prioridad==2 && random==1){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,245,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;

                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==2 && random==2){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,306,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==2 && random==3){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,371,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }



            if (prioridad==3 && random==1){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,245,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }

                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==3 && random==2){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,306,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==3 && random==3){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,371,zso,vel,40,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                    if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){

                        return;
                    }
                    return;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }


            banderaEstadoEjec=false;
        }
qDebug("despues while");
    }

detenerEjecucion();
}





void ATModelo::Eject2(){
    int prioridad=1;
    int random=1;
    qDebug()<<"Iniciando Ejecucion";
    robotEjecucion = new ATRobot(PUERTO_EJECUCION);
    //Ir a la posicion de espera
    robotEjecucion->salidasActuadorPLC(0,false);

    bool banderaEstadoEjec=true;




    bool inicio =true;
    int vel=650;
    int acel=0.5;
    int zp=952;
    int zso = 920;
    int primerpaso=0;
    int memoria;

    prioridad =0;
    int c1=0;
    int c2=0;
    int c3=0;
    random =0;
    int ze=870; // CUIDADO CUIDADO CUIDADO
    int radio=60;

    while (prioridad==0){
        sen1=robotEjecucion->statusPinEntrada(5);
        sen2=robotEjecucion->statusPinEntrada(4);
        sen3=robotEjecucion->statusPinEntrada(3);
        if(sen1==true){
            c1+=1;
        }
        else if(sen2==true){
            c2+=1;
        }
        else if(sen3==true){
            c3+=1;
        }
        if(c1>0){
            prioridad=1;
        }
        else if(c2>0){
            prioridad=2;
        }
        else if(c3>0){
            prioridad=3;
        }
        if(getStopReceived()){
           break;
        }

    }


    switch(prioridad){
    case 1:
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,16,775,vel) != COMANDO_EJECUTADO){

           break;

        }
        if(getStopReceived()){
           break;
        }
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,16,zp,vel) != COMANDO_EJECUTADO){
          break;
        }
        robotEjecucion->salidasActuadorPLC(0,true);
        c1-=1;
        if(getStopReceived()){
           break;
        }

        primerpaso=1;
        break;
    case 2:
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-51,775,vel) != COMANDO_EJECUTADO){
            break;

        }
        if(getStopReceived()){
           break;
        }
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-51,zp,vel) != COMANDO_EJECUTADO){
            break;
        }
        c2-=1;
        robotEjecucion->salidasActuadorPLC(0,true);
        if(getStopReceived()){
           break;
        }
        primerpaso=1;
        break;
    case 3:
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-120,775,vel) != COMANDO_EJECUTADO){
            break;
        }
        if(getStopReceived()){
           break;
        }
        if(robotEjecucion->setPunto(MOV_LINEAL,-121,-120,zp,vel) != COMANDO_EJECUTADO){
            break;
        }
        c3-=1;
        robotEjecucion->salidasActuadorPLC(0,true);
        if(getStopReceived()){
           break;
        }
        primerpaso=1;
        break;
    }
qDebug("antes while");
    while (inicio){

        random = 1+rand()%3;
        banderaEstadoEjec=true;
        do{
            if (primerpaso==1){
                break;
            }
            sen1=robotEjecucion->statusPinEntrada(5);
            sen2=robotEjecucion->statusPinEntrada(4);
            sen3=robotEjecucion->statusPinEntrada(3);

            if(sen1==true){
                c1+=1;
            }
            else if(sen2==true){
                c2+=1;
            }
            else if(sen3==true){
                c3+=1;
            }
            if(c1>0){
                prioridad=1;
                break;
            }
            else if(c2>0){
                prioridad=2;
                break;
            }
            else if(c3>0){
                prioridad=3;
                break;
            }



            if(getStopReceived()){
               break;
            }
            if(sen1==false && sen2==false && sen3==false){
                prioridad=0;

            }


        }while(prioridad==0);

        if(getStopReceived()){
           break;
        }
        if(primerpaso==1){
            if (primerpaso==1){
                memoria=prioridad;
                prioridad=0;
                primerpaso=0;
                if(c1>=1){
                    c1-=1;
                }
                else if(c2>=1){
                    c2-=1;
                }
                else if(c3>=1){
                    c3-=1;
                }

            }
        }

        switch(prioridad){
        case 1:
            if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-121,16,zp,vel,radio,acel,ze) != COMANDO_EJECUTADO){

                break;
            }
            c1-=1;
            robotEjecucion->salidasActuadorPLC(0,true);
            if(getStopReceived()){
               break;
            }

            break;
        case 2:if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-121,-51,zp,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                //detenerEjecucion();
                break;
            }
            c2-=1;
            robotEjecucion->salidasActuadorPLC(0,true);
            if(getStopReceived()){
               break;
            }

            break;
        case 3: if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-121,-120,zp,vel,radio,acel,ze) != COMANDO_EJECUTADO){

                break;
            }
            c3-=1;
            robotEjecucion->salidasActuadorPLC(0,true);
            if(getStopReceived()){
               break;
            }
        case 0:
            prioridad=memoria;
            break;
        }
        while (banderaEstadoEjec==true){
            if (prioridad==1 && random==1){
               qDebug("aquir1");
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,245,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    // banderaEstadoEjec=false; poner bandera de estado para el while oh no o.O
                    break;
                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);

            }

            else if (prioridad==1 && random==2){
                qDebug("aquir2");
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,308,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==1 && random==3){
               qDebug("aquir3");
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,376,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }


            if (prioridad==2 && random==1){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,245,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;

                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==2 && random==2){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,308,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==2 && random==3){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,376,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }



            if (prioridad==3 && random==1){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,245,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }

                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==3 && random==2){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,308,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            else if (prioridad==3 && random==3){
                if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,5,376,zso,vel,radio,acel,ze) != COMANDO_EJECUTADO){
                    banderaEstadoEjec=false;
                    break;
                }
                if(getStopReceived()){
                   break;
                }
                robotEjecucion->salidasActuadorPLC(0,false);
            }

            if(getStopReceived()){
               break;
            }
            banderaEstadoEjec=false;
        }
        if(getStopReceived()){
           break;
        }
    }

    if(getStopReceived()){
      detenerEjecucion();
    }
}






void ATModelo::Ejecucion(){

    qDebug()<<"Iniciando Ejecucion";
    robotEjecucion = new ATRobot(PUERTO_EJECUCION);
    banderaEstadoEjec = true;
    //Ir a la posicion de espera
    aPosicionEspera(robotEjecucion);
    robotEjecucion->salidasActuadorPLC(0,false);
    double z=970;


    if(robotEjecucion->setPunto(MOV_LINEAL,0,200,875,300) != COMANDO_EJECUTADO){
        detenerEjecucion();
        return;
    }

    if(getStopReceived()){
        detenerEjecucion();
        return;
    }

    while(banderaEstadoEjec){
        if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,0,-200,875,300,40,40,775) != COMANDO_EJECUTADO){
            banderaEstadoEjec=false;
            break;
        }

        qDebug()<<"Estado pin3 "<<sen1;
        qDebug()<<"Estado pin4 "<<sen2;
        qDebug()<<"Estado pin5 "<<sen3;

        if(getStopReceived()){
            banderaEstadoEjec=false;
            break;
        }

        if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,0,200,875,300,40,40,775) != COMANDO_EJECUTADO){
            banderaEstadoEjec=false;
            break;
        }
        qDebug()<<"Estado pin3 "<<sen1;
        qDebug()<<"Estado pin4 "<<sen2;
        qDebug()<<"Estado pin5 "<<sen3;

        if(getStopReceived()){
            banderaEstadoEjec=false;
            break;
        }
    }




    detenerEjecucion();

}


void ATModelo::Ejecucion3(){

    qDebug()<<"Iniciando Ejecucion";
        robotEjecucion = new ATRobot(PUERTO_EJECUCION);
        banderaEstadoEjec = true;
        //Ir a la posicion de espera
        aPosicionEspera(robotEjecucion);
    robotEjecucion->salidasActuadorPLC(0,false);
    double z=970;
        do {
            double dx=40;
                    double dy=22;
                    double posXcol=-238;
                    double posYcol=99;

                    double pxc2=256;
                    double pyc2=101;
                   if(robotEjecucion->setPunto(MOV_LINEAL,0,0,775,300) != COMANDO_EJECUTADO){
                        break;
                    }
                    if(robotEjecucion->setPunto(MOV_LINEAL,posXcol,posYcol,847,300) != COMANDO_EJECUTADO){
                        break;
                    }

                    if(robotEjecucion->setPunto(MOV_LINEAL,posXcol,posYcol,z,300) != COMANDO_EJECUTADO){
                        break;
                    }
                    robotEjecucion->salidasActuadorPLC(0,true);

                    for(int n1=3; n1>0; n1--){
                    double posYfila=posYcol;
                    double pyf2=pyc2;


                        for(int n2=n1; n2>0; n2--){
                            if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,pxc2,pyf2,z-10,300,40,40,700) != COMANDO_EJECUTADO){
                                break;
                        }
                            robotEjecucion->salidasActuadorPLC(0,false);


                            posYfila-=(2*dy);
                            pyf2-=(2*dy);
                            if(n2!=1){


                            if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,posXcol,posYfila,z,300,40,40,700) != COMANDO_EJECUTADO){
                                break; // en for anidado, detener ejecucion y efector
                        }
                            robotEjecucion->salidasActuadorPLC(0,true);
                            }


                            if(getStopReceived()){
                                robotEjecucion->salidasActuadorPLC(0,false);
                                break;
                            }
                    }
                        if(getStopReceived()){
                            break;
                            robotEjecucion->salidasActuadorPLC(0,false);
                        }

                        posXcol+=dx;
                        posYcol-=dy;
                        pyc2-=dy;
                        pxc2-=dx;
                        if(n1!=1){
                        if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,posXcol,posYcol,z,300,40,40,700) != COMANDO_EJECUTADO){
                            break;
                        }
                        robotEjecucion->salidasActuadorPLC(0,true);
                        }

                    }



                    //Regreso
                    dx=40;
                    dy=22;
                    posXcol=-238;
                    posYcol=99;

                    pxc2=256;
                    pyc2=101;

                    if(robotEjecucion->setPunto(MOV_LINEAL,pxc2,pyc2,847,300) != COMANDO_EJECUTADO){
                        break;
                    }
                    if(robotEjecucion->setPunto(MOV_LINEAL,pxc2,pyc2,z,300) != COMANDO_EJECUTADO){
                        break;
                    }
                    robotEjecucion->salidasActuadorPLC(0,true);
                    for(int n1=3; n1>0; n1--){
                    double posYfila=posYcol;
                    double pyf2=pyc2;


                        for(int n2=n1; n2>0; n2--){
                            if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,posXcol,posYfila,z-10,300,40,40,700) != COMANDO_EJECUTADO){
                                break;
                        }
                            robotEjecucion->salidasActuadorPLC(0,false);


                            posYfila-=(2*dy);
                            pyf2-=(2*dy);
                            if(n2!=1){


                            if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,pxc2,pyf2,z,300,40,40,700) != COMANDO_EJECUTADO){
                               break; // en for anidado, detener ejecucion y efector
                        }
                            robotEjecucion->salidasActuadorPLC(0,true);
                            }


                            if(getStopReceived()){
                                robotEjecucion->salidasActuadorPLC(0,false);
                                break;
                            }
                    }
                        if(getStopReceived()){
                            break;
                            robotEjecucion->salidasActuadorPLC(0,false);
                        }

                        posXcol+=dx;
                        posYcol-=dy;
                        pyc2-=dy;
                        pxc2-=dx;
                        if(n1!=1){
                        if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,pxc2,pyc2,z,300,40,40,700) != COMANDO_EJECUTADO){
                            break;
                        }
                        robotEjecucion->salidasActuadorPLC(0,true);
                        }

                    }




            if(getStopReceived()){
                break;
            }
        }while(banderaEstadoEjec);

        detenerEjecucion();

    }



void ATModelo::Ejecucion2(){
    qDebug()<<"Iniciando Ejecucion";
    robotEjecucion = new ATRobot(PUERTO_EJECUCION);
    //banderaEstadoEjec = true;
    //Ir a la posicion de espera
    aPosicionEspera(robotEjecucion);
robotEjecucion->salidasActuadorPLC(0,false);

    do {
    if(robotEjecucion->setPunto(MOV_LINEAL,-186,55,847,300) != COMANDO_EJECUTADO){
        break;
    }
    if(robotEjecucion->setPunto(MOV_LINEAL,-186,55,940,300) != COMANDO_EJECUTADO){
        break;
    }
    robotEjecucion->salidasActuadorPLC(0,true);
    if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,170,67,947,300,40,40,700) != COMANDO_EJECUTADO){
        break;
    }
     robotEjecucion->salidasActuadorPLC(0,false);
     if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-196,77,970,300,40,40,700) != COMANDO_EJECUTADO){
         break;
     }
     robotEjecucion->salidasActuadorPLC(0,true);

    if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,208,84,947,300,40,40,700) != COMANDO_EJECUTADO){
        break;
    }
    robotEjecucion->salidasActuadorPLC(0,false);
    if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-196,33,970,300,40,40,700) != COMANDO_EJECUTADO){
        break;
    }
    robotEjecucion->salidasActuadorPLC(0,true);
    if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,208,40,947,300,40,40,700) != COMANDO_EJECUTADO){
        break;
    }
    robotEjecucion->salidasActuadorPLC(0,false);
    if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,-158,55,970,300,40,40,700) != COMANDO_EJECUTADO){
        break;
    }
    robotEjecucion->salidasActuadorPLC(0,true);
    if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,190,57,940,300,40,40,700) != COMANDO_EJECUTADO){
        break;
    }
   robotEjecucion->salidasActuadorPLC(0,false);
   if(robotEjecucion->setPunto(MOV_TRAYECTORIA_U,0,0,775,300,40,40,700) != COMANDO_EJECUTADO){
       break;
   }

    if(getStopReceived()){
        break;
    }
    }while(banderaEstadoEjec);

    detenerEjecucion();
   }

void ATModelo::setStopReceived(bool estado){
    if(estado){
        robot->softStop();
        ATUtils::utils_espera<std::chrono::seconds>(2);
        //robot->getPosActual(&xReanudar,&yReanudar,&zReanudar);
    }
    mutexSenalParo.lock();
    stopReceived = estado;
    mutexSenalParo.unlock();
}

void ATModelo::conectarRobot(){
    robot = new ATRobot(PUERTO_CALIBRACION);
    if(!robot->isServerConnected()){
        emit errorDeConexion(PUERTO_CALIBRACION);
        return;
    }

    robot->limpiarErrores();

    if(!robot->statusPinEntrada(PIN_PARO)){
        estadoParoInicio = true;
        emit errorParo();
    }
    inicializarRobot();
    //bloquear con mutex
    socketPrincipalConectado = true;
}

void ATModelo::lanzarAlarmaMonitor(){
    QFuture<void>future;
    future = QtConcurrent::run(this,&ATModelo::alarmaMonitor);
}

void ATModelo::detenerEjecucion(){
    reanudar = true;
    //    banderaEstadoEjec = false;
    mutexSenalParo.lock();
    stopReceived = false;
    mutexSenalParo.unlock();
    pararEfector();
    emit errorMovimiento();
    robotEjecucion->desconectar();
    delete robotEjecucion;
}

/*Metodos privados*/
bool ATModelo::comprobarErrores(ATRobot *robotAux, int * repeticiones){
    if(robotAux->comprobarErrores(WARNING) && *repeticiones < NUM_MAX_REINTENTOS_INICIO){
        robotAux->limpiarErrores();
        *repeticiones += 1;
        return true;
    }
    if(robotAux->comprobarErrores(CRITICAL) && *repeticiones < NUM_MAX_REINTENTOS_INICIO){
        robotAux->limpiarErrores();
        *repeticiones += 1;
        return true;
    }
    return false;
}

int ATModelo::comprobarComando(ATRobot *robotAux, int valor, int *repeticiones, int puertoSocket){
    if(valor == ERROR_COMUNICACION){
        errorDeConexion(puertoSocket);
        return SALIR;
    }
    else if(valor == ERROR_COMANDO_NO_EJECUTADO){
        *repeticiones += 1;
        robotAux->limpiarErrores();
        return REINTENTAR;
    }
    return COMANDO_COMPLETADO;
}

void ATModelo::monitorIos(){// 3 4 y 5 sensores
    robotIos = new ATRobot(PUERTO_IOS);
    qDebug()<<"Hilo de Ios instaciado";
    while(!getStopReceivedIos()){
        ATUtils::utils_espera<std::chrono::milliseconds>(500);
        //qDebug()<<"Pin entrada "<<robotIos->statusPinEntrada(16);
        //qDebug()<<"Estado pin3 "<<robotIos->statusPinEntrada(3);
        //qDebug()<<"Estado pin4 "<<robotIos->statusPinEntrada(4);
        //qDebug()<<"Estado pin5 "<<robotIos->statusPinEntrada(5);
        sen1=robotIos->statusPinEntrada(5);
        sen2=robotIos->statusPinEntrada(4);
        sen3=robotIos->statusPinEntrada(3);
       // qDebug()<<"Salidas "<<robotIos->getEstadoSalidas();
//        if(!robotIos->statusPinEntrada(15)){
//            qDebug()<<"estado del sensor apqagado";
//        }

    }
    robotIos->desconectar();
    delete robotIos;
}

void ATModelo::alarmaMonitor(){
    QThread::currentThread()->setPriority(QThread::TimeCriticalPriority);

    do{
        ATUtils::utils_espera_process_events(100);
    }while(!socketPrincipalConectado);

    robotParo           = new ATRobot(PUERTO_PARO);
    robotParoFinalizado = false;
    salida              = false;

    u_int16_t read; //buffer donde se guarda la respuesta del PLC
    u_int16_t maxread       = -1;
    bool errorReanudacion   = false;
    int numIntentosError    = 0, valor = 0;

    if(estadoParoInicio){
        robotParo->desconectar();
        robotParoFinalizado = true;
        delete robotParo;
        return;
    }

    do{
        read = maxread;
        read = robotParo->leer(TIEMPO_LECTURA_MS);

        if(read == PARO_FISICO){
            emit enParo(true);
            QCoreApplication::processEvents(QEventLoop::AllEvents,100);

            mutexSenalParo.lock();
            stopReceived = true;
            mutexSenalParo.unlock();
            ATUtils::utils_espera<std::chrono::milliseconds>(500);
            robotParo->limpiarErrores();

secuenciaReanudacion:

            if(numIntentosError >= NUM_MAX_REINTENTOS_INICIO){
                emit enParo(false);
                stopReceived = false;
                emit errorDeConexion(PUERTO_PARO);
                continue;
            }

            if(!errorReanudacion){
                valor = robotParo->plcReady(true);
                switch (comprobarComando(robotParo,valor,&numIntentosError,PUERTO_PARO)) {
                case SALIR:
                    errorDeConexion(PUERTO_PARO);
                    return;
                case REINTENTAR:
                    errorReanudacion = false;
                    goto secuenciaReanudacion;
                }
                valor = robotParo->homming(true);
                switch (comprobarComando(robotParo,valor,&numIntentosError,PUERTO_PARO)) {
                case SALIR:
                    errorDeConexion(PUERTO_PARO);
                    return;
                case REINTENTAR:
                    errorReanudacion = false;
                    goto secuenciaReanudacion;
                }
            }
            errorReanudacion = false;

            if(comprobarErrores(robotParo,&numIntentosError))
                goto secuenciaReanudacion;

            valor = robotParo->sync(SYNC_FORZADO);
            switch (comprobarComando(robotParo,valor,&numIntentosError,PUERTO_PARO)) {
            case SALIR:
                if(!reconectar(PUERTO_PARO)){
                    emit enParo(false);
                    stopReceived = false;
                    emit errorDeConexion(PUERTO_PARO);
                }
                robotParo->sync(SYNC_OFF);
                goto secuenciaReanudacion;
                return;
            case REINTENTAR:
                errorReanudacion = false;
                goto secuenciaReanudacion;
            }

            if(comprobarErrores(robotParo,&numIntentosError))
                goto secuenciaReanudacion;


            u_int16_t respuestaPostParo = -1;

            //Espera a que se libere el paro
            do{
                respuestaPostParo = -1;
                ATUtils::utils_espera_process_events(100,QEventLoop::ExcludeUserInputEvents);
                respuestaPostParo = robotParo->leer();

            }while(respuestaPostParo != PARO_FISICO_LIBERADO);

            stopReceived = false;
            emit enParo(false);

        }
    }while(!salida);
    robotParo->desconectar();
    delete robotParo;

    robotParoFinalizado = true;
}


void ATModelo::aPosicionEspera(ATRobot *robotE){
    double x,y,z;
    robotE->getPosActual(&x,&y,&z);
    bool inHome = enHome(x,y,z);

    if(!inHome){
        robotE->setPunto(MOV_LINEAL,x,y,posicionSegura.z,300);
        robotE->setPunto(MOV_LINEAL,posicionSegura.x,posicionSegura.y,
                        posicionSegura.z,300);
    }

    else{
        robotE->setPunto(MOV_LINEAL,posicionSegura.x,posicionSegura.y,
                        posicionSegura.z,300);
    }
}

bool ATModelo::reconectar(int numSocket){
    if(numSocket == 1){
        robot->desconectar();
        delete robot;
        robot = new ATRobot(8091);

        robot->limpiarErrores();
        ATUtils::utils_espera_process_events(200);
        if(robot->getEstadoSocket() == UnconnectedState)
            return false;
    }
    else if(numSocket == 3){
        robotParo->desconectar();
        delete robotParo;
        robotParo = new ATRobot(8093);

        robotParo->limpiarErrores();
        ATUtils::utils_espera_process_events(200);

        if(robotParo->getEstadoSocket() == UnconnectedState)
            return false;
    }
    return true;
}

bool ATModelo::enHome(double x,double y,double z){
    double tolerancia = 10;

    return (sqrt( pow(x - posicionSegura.x,2) +
                  pow(y - posicionSegura.y,2) +
                  pow(z - posicionSegura.z,2) )
            <= tolerancia) ? true : false;
}

void ATModelo::pararEfector(){
    robotEjecucion->salidasActuadorPLC(0,false);
}
