#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>
#include <vector>
#include <SerialStream.h>
#include <cmath>

using namespace std;
using namespace LibSerial;

int main(int argc, char **argv)
{
    string dev= "/dev/ttyACM0";
    SerialStream serial_port(dev,ios_base::in | ios_base::out);
    serial_port.SetBaudRate(SerialStreamBuf::BAUD_9600);
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8);
    char numx,numy,ch=0;
    int nx,ny,conn=0,snx=0,sny=0;
	cout<<"inicia";

	//Declarando setpoint, posicion actual, errores, velocidad.

	//max spx=352
	float spx=194,valx=0,errx=0,velx=0,ldx=0,dx=0,cix=0,Kx;
	//max spy=288
	float spy=137,valy=0,erry=0,vely=0,ldy=0,dy=0,ciy=0,Ky;

	// Parametros de camara, cambiar el indice de la camara si se quiere utilizar otro dispositivo.
	cv::Mat frame;
	int index =1;
	cv::VideoCapture cap;
	if (!cap.open(index))
	{
		cout << "Camara web no conectada.\n" << "Verifique por favor.\n";
		return EXIT_FAILURE;
	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 352);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 288);

	cout << "\nq to exit...\n";

    //Rango y controlador
    int Rint=1,Rext=16,aux1,aux2,aux3,aux4,aux5;

    float kpx=1.757;
    float kix=0.0;
    float kdx=40.549;

    float kpy=1.757;
    float kiy=0.0;
    float kdy=40.549;


	// Bucle principal
	while (ch != 'q' && ch != 'Q')
	{
        // Adquisicion de imagen
		cap >> frame;

		cv::Mat res;
		frame.copyTo(res);

		// Reducir ruido
		cv::Mat blur;
		cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);

		// Conversion HSV
		cv::Mat frmHsv;
		cv::cvtColor(blur, frmHsv, CV_BGR2HSV);

		// Cambiar parametros para diferentes colores
		cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
		cv::inRange(frmHsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 20), rangeRes);


		// Para mejorar resultados
//		cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
//		cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);

		// Ver el threshold
		cv::imshow("Threshold", rangeRes);

    // Deteccion de contornos------------------------------------------------------------------------------------------------------------
		vector<vector<cv::Point> > contours;
		cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

		// Filtrando objetos
		vector<vector<cv::Point> > balls;
		vector<cv::Rect> ballsBox;
		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::Rect bBox;
			bBox = cv::boundingRect(contours[i]);
			float ratio = (float)bBox.width / (float)bBox.height;
			if (ratio > 1.0f)
			{
				ratio = 1.0f / ratio;
            }
			if (ratio > 0.75 && bBox.area() >= 400 && bBox.area()<=5000)//No detecta esferas muy grandes
			{
				balls.push_back(contours[i]);
				ballsBox.push_back(bBox);
				break;
			}
		}
		conn++;
		if(balls.size()==0)
        {
            if((conn%3)==0)
            {
                conn=1;
                nx=45;
                numx=nx;
                if(nx!=snx)
                {
                    serial_port <<numx;
                    snx=nx;
                }
            }
            else if(((conn+1)%3)==0)
            {
                ny=135;
                numy=ny;
                if(ny!=sny)
                {
                    serial_port <<numy;
                    sny=ny;
                }
            }
            valx=spx+errx/0.13;
            valy=spy+erry/0.13;
        }

		for (size_t i = 0; i < balls.size(); i++)
		{
			cv::drawContours(res, balls, i, CV_RGB(20, 150, 20), 1);
			cv::rectangle(res, ballsBox[i], CV_RGB(0, 255, 0), 2);
			cv::Point center;
			center.x = ballsBox[i].x + ballsBox[i].width / 2;
			center.y = ballsBox[i].y + ballsBox[i].height / 2;
			cv::circle(res, center, 2, CV_RGB(20, 150, 20), -1);
			valx=center.x;
			valy=center.y;

            //Hallando errores en cm
            errx=(valx-spx)*0.13;
            erry=(valy-spy)*0.13;


//------------------------------------------------------------------------------------------------------------
            ldx = dx; dx = errx;
            ldy = dy; dy = erry;
            if ((abs(dx) > Rint) && (abs(dx) < Rext))
            {
                cix+=dx;
            }
            else
            {
                cix=0;
            }

            if ((abs(dy) > Rint) && (abs(dy) < Rext))
            {
                ciy+=dy;
            }
            else
            {
                ciy=0;
            }

            velx=(dx-ldx);
            vely=(dy-ldy);
            if(dx*(dx+velx)>0)
            {
                Kx=kpx*(dx+velx)+kix*cix+kdx*velx;
            }
            else
            {
                Kx=20*(Kx)/fabs(Kx);
            }
            if(dy*(dy+vely)>0)
            {
                Ky=kpy*(dy+vely)+kiy*ciy+kdy*vely;
            }
            else
            {
                Ky=20*(Ky)/fabs(Ky);
            }
            aux1=dy;
            aux2=ciy;
            aux3=vely;
            aux4=Kx;
            aux5=Ky;

            if((conn%3)==0)
            {
                if(fabs(Kx)<30)
                {
                    nx=45+Kx*0.5;
                    numx=nx;
                    if(abs(nx-snx)>5)
                    {
                        serial_port <<numx;
                        snx=nx;
                        cout<<"   motor2: "<<nx<<" / "<<Kx<<endl;
                    }
                }
                else
                {
                    nx=135+Kx*10/fabs(Kx)+Kx/10;
                    numx=nx;
                    if(abs(nx-snx)>5)
                    {
                        serial_port <<numx;
                        snx=nx;
                        cout<<"   motor2: "<<nx<<" / "<<Kx<<endl;
                    }
                }
            }
            else
            {
                if(((conn+1)%3)==0)
                {
                    if(fabs(Ky)<30)
                    {
                        ny=135+Ky*0.5;
                        numy=ny;
                        if(abs(ny-sny)>5)
                        {
                            serial_port <<numy;
                            sny=ny;
                            cout<<"   motor2: "<<ny<<" / "<<Ky<<endl;
                        }
                    }
                    else
                    {
                        ny=135+Ky*10/fabs(Ky)+Ky/10;
                        numy=ny;
                        if(abs(ny-sny)>5)
                        {
                            serial_port <<numy;
                            sny=ny;
                            cout<<"   motor2: "<<ny<<" / "<<Ky<<endl;
                        }
                    }
                }
            }
            stringstream sstr;
            sstr <<"(" << errx << "," << erry << ")";
            cv::putText(res, sstr.str(),cv::Point(center.x + 3, center.y - 3),cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20, 150, 10), 2);

            //CAmbia luego
            stringstream nPID;
            cout <<"("<<aux1<<","<<aux2<<","<<aux3<<","<<aux4<<") "<<"("<<errx<<","<<erry<<") ";
            nPID <<"("<<aux4<<","<<aux5<<") "<<"("<<errx<<","<<erry<<") ";
            cv::putText(res, nPID.str(),cv::Point(5, 15),cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20, 150, 10), 2);
        }

		// Mostrar deteccion.
		cv::imshow("Tracking", res);
		// User key
		ch = cv::waitKey(1);
	}
	serial_port.Close();
	return EXIT_SUCCESS;
}
