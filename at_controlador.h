#ifndef ATCONTROLADOR_H
#define ATCONTROLADOR_H

#include <QObject>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <lib_ATGUI/at_mensaje.h>
#include "at_vista.h"
#include "at_modelo.h"
#include "estado_programa.h"

class ATControlador : public QObject
{
    Q_OBJECT
public:
    explicit ATControlador(QObject *parent = 0);
    ATControlador(ATVista* vista,ATModelo* modelo);

signals:
    void salida(bool opcion);
    void senalParo(bool);
    void conectarRobot();
public slots:

private:
    ATMensaje* mensaje;
    ATVista *  vista;
    ATModelo* modelo;
    QThread * hiloModelo;

    QFuture<void>future;
    QFuture<void>futureIos;

    QString mensajeDescriptivo;
    QString rutaImaBtnAceptarMen;
    bool ejecucionIniciada;
    bool virtualStop;

    void initVistas();
    void initConexionesVista();
    void initConexionesVistaAviso();
    void initConexionesModelo();
    void initParametros();
    void blockGUI(bool opcion);
    void secuenciaInicio();
    void setErrorConexion(int puerto);
    void salirEjecucion();
    void enParo(bool opcion);
private slots:
    void cerrarAplicacion();
    void clickedStartStop();
    void setStop();
    void setErrorParo();
    void liberarApi(int);
};

#endif // ATCONTROLADOR_H
