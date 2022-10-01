using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
public class FishModel : MonoBehaviour
{//Alvaro 2007
    public GameObject[] articulated_body;
    public double U;
    public double omega, kappa;
    public double timestep;
    public double pulse;
    public double time;
    public double R1,R2,R3,R4;
    public double r1,r2,r3,r4;
    public double l;
    public double H0;
    public int xres;
    public double[] h;
    public double[] hdot;
    public double a0;
    public double mean_lift;
    public double x0;
    public double thetaaccel, xaccel, zaccel;
    public double thetavel, xvel,yvel;
    public double theta, x, y;
    public double total_mass;
    public double noise;
    public double rho;
    public double payload_mass;
    public double a;
    public double E, mu, M;
    public double costheta;
    public double tscale;
    public double Cd;
    public double pf;
    public double m(double x){
    	return 3.1415927*rho*Math.Pow(2*R(x),2);
    }
    public void InitParams(){//from Tuna model, table 5.1
    	E=48624;
    	mu=25.8;
    	rho=1000;
    	l=0.3;
    	a0=0.5*l;
    	a=0.18;
    	M=1.4;
    	kappa=2*3.1415927/1.1/l;
    	x0=0;
    	H0=0.002;
    	R1=0.1*l;
    	R2=2*3.1415927/1.57/l;
    	R3=8e-5/1.57/l;
    	R4=2*3.1415927/0.81/l;
    	r1=0.055*l;
    	r2=2*3.1415927/1.25/l;
   	r3=0.08*l;
    	r4=2*3.1415927/3.14/l;
    	total_mass+=payload_mass;
    	for(int i=0; i<xres; ++i){
	    	total_mass+=rho*A(l*i/xres)/xres*l;
    		total_mass+=m(l*i/xres)/xres*l;
    	}
    	h=new double[xres];
    	hdot=new double[xres];
    	for(int i=0; i<xres; ++i){
    		h[i]=0.0;
    		hdot[i]=0.0;
    	}
    }
    public double R(double x){
    	return R1*Math.Sin(R2*x)+R3*(Math.Exp(R4*x)-1);
    }
    public double dRdx(double x){
    	return R1*R2*Math.Cos(R2*x)+R3*R4*(Math.Exp(R4*x));
    }
    public double r(double x){
    	return r1*Math.Sin(r2*x)+r3*Math.Sin(r4*x);
    }
    public double u(double x){
	return 1.0/(1.0+Math.Exp(-5.0*x));
    }
    public double A(double x){
    	return 3.1415927*R(x)*r(x);
    }
    
    public double I(double x){
    	return 3.1415927/4*R(x)*Math.Pow(r(x),3);
    }
    
    public double dmdx(double x){
    	return 3.1415927*4*rho*2*R(x)*dRdx(x);
    }
    public double op1(int i,double t, double[] k){
    	int N=k.Length/2;
    	if(i==0){
    		return op1(i+1,t,k);
    	}
    	if(i==N-1){
    		return op1(i-1,t,k);
    	}
    	double x=((l-x0)/(double)N)*(double)i+x0;
    	double hxx=(k[i+1]+k[i-1]-2*k[i])/Math.Pow(l/(double)N,2);
	double hdotxx=(k[i+1+N]+k[i-1+N]-2*k[i+N])/Math.Pow(l/(double)N,2);
	return (E*I(x)*hxx+mu*I(x)*hdotxx)-M*Math.Sin(omega*t)*u(x-a);
    }
    public double hdotdot(int i,double t,double[] k){
      	int N=k.Length/2;
  	double x=((l-x0)/(double)N)*(double)i+x0;
 	double xn=((l-x0)/(double)N)*(double)(i-1)+x0;
 	double xp=((l-x0)/(double)N)*(double)(i+1)+x0;
    	if(i==0){
    		return -H0*omega*omega*Math.Sin(omega*t+kappa*x0);
    	}
    	else if(i==N-1){
    		return hdotdot(i-1,t,k);
    	}
    	else{
    		double opxx=(op1(i+1,t,k)+op1(i-1,t,k)-2*op1(i,t,k))/Math.Pow(l/(double)N,2);
    		double hdotx=(k[i+1+N]-k[i-1+N])/(l/(double)N);	
   		double hx=(k[i+1]-k[i-1])/(2*l/(double)N);	
	    	double hxx=(k[i+1]+k[i-1]-2*k[i])/Math.Pow(l/(double)N,2);
	    	return (opxx+2*U*m(x)*hdotx+2*U*dmdx(x)*hx-U*U*m(x)*hxx+0.5*U*U*pf*Cd*A(x))/(-rho*A(x)-m(x));
	}
    }
    public double deriv(int i,double t,double[] k){
	if(i>=k.Length/2){
		return hdotdot(i-k.Length/2,t,k);
	}
	else{
		return k[i+k.Length/2];
	}
    }
    
    public double lift(int i,double t,double[] k){
    		int N=k.Length/2;
	  	double x=((l-x0)/(double)N)*(double)i+x0;
   		double hdd=this.hdotdot(i,t,k);
   		double hdotx=(k[i+1+N]-k[i-1+N])/(l/(double)N);	
   		double hx=(k[i+1]-k[i-1])/(2*l/(double)N);	
	    	double hxx=(k[i+1]+k[i-1]-2*k[i])/Math.Pow(l/(double)N,2);
	    	return m(x)*hdd+2*U*m(x)*hdotx+2*U*dmdx(x)*hx+U*U*m(x)*hxx;
    }
    public void integrate(double t, double step){
    	double[] k1, k2, k3, k4;
   	double[] y1, y2, y3, y4;
   	int N=h.Length;
    	k1=new double[N*2];
   	k2=new double[N*2];
   	k3=new double[N*2];
   	k4=new double[N*2];
    	y1=new double[N*2];
   	y2=new double[N*2];
   	y3=new double[N*2];
   	y4=new double[N*2];
   	double a1,a2,a3,a4,b1,b2,b3,b4,c1,c2,c3,c4;
   	double u1,u2,u3,u4,v1,v2,v3,v4,w1,w2,w3,w4;
  	double z1,z2,z3,z4,x1,x2,x3,x4,theta1,theta2,theta3,theta4;
   	for(int i=0; i<N; ++i){
   		y1[i]=h[i];
   		y1[i+N]=hdot[i];	
   	}
   		mean_lift=0;
	   	for(int i=1; i<N-1; ++i){
	   		mean_lift+=lift(i,t,y1)/(N-2);
	   	}
   		a1=mean_thrust(y1)/total_mass;
		b1=mean_lift/total_mass;
		u1=xvel+a1*step/2;
		v1=yvel+b1*step/2;
		x1=x+u1*step/2;
		z1=y+v1*step/2;

   	for(int i=0; i<N*2; ++i){
   		k1[i]=deriv(i,t,y1);
   		y2[i]=y1[i]+k1[i]*step/2;
   	}
  		mean_lift=0;
	   	for(int i=1; i<N-1; ++i){
	   		mean_lift+=lift(i,t+step/2,y2)/(N-2);
	   	}
  		a2=mean_thrust(y2)/total_mass;
		b2=mean_lift/total_mass;
		u2=xvel+a2*step/2;
		v2=yvel+b2*step/2;
		x2=x+u2*step/2;
		z2=y+v2*step/2;
   	
   	
   	for(int i=0; i<N*2; ++i){
   		k2[i]=deriv(i,t+step/2,y2);
   		y3[i]=y1[i]+k2[i]*step/2;
   	}
   		mean_lift=0;
	   	for(int i=1; i<N-1; ++i){
	   		mean_lift+=lift(i,t+step/2,y3)/(N-2);
	   	}
  		a3=mean_thrust(y3)/total_mass;
		b3=mean_lift/total_mass;
		u3=xvel+a3*step/2;
		v3=yvel+b3*step/2;
		x3=x+u2*step/2;
		z3=y+v2*step/2;
   	
   	
   	for(int i=0; i<N*2; ++i){
   		k3[i]=deriv(i,t+step/2,y3);
   		y4[i]=y1[i]+k3[i]*step;
   	}	
  		a4=mean_thrust(y4)/total_mass;
  		
   		mean_lift=0;
	   	for(int i=1; i<N-1; ++i){
	   		mean_lift+=lift(i,t+step/2,y4)/(N-2);
	   	}
		b4=mean_lift/total_mass;
		u4=xvel+a4*step;
		v4=yvel+b4*step;
		x4=x+u4*step;
		z4=y+v4*step;
   	
   	
   	for(int i=0; i<N*2; ++i){
   		k4[i]=deriv(i,t+step,y4);
   		y1[i]+=step/6*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
   	}
   	
   	for(int i=0; i<N; ++i){
   		h[i]=y1[i];
   		hdot[i]=y1[i+N];
   	};
   	mean_lift=0;
   	for(int i=1; i<N-1; ++i){
   		mean_lift+=lift(i,t+step,y1)/(N-2);
   	}
   	zaccel=mean_thrust(y1)/total_mass;
	xaccel=mean_lift/total_mass;
	xvel+=(a1+2*a2+2*a3+a4)/6*step;
	yvel+=(b1+2*b2+2*b3+b4)/6*step;
	x+=(u1+2*u2+2*u3+u4)/6*step;
	y+=(v1+2*v2+2*v3+v4)/6*step;
   }
   public double mean_thrust(double[] k){
   	int N=h.Length;
   	return m(l)*omega*(k[N+N-1]-omega/2);
   }
   
    // Start is called before the first frame update
    void Start()	
    {
    }    
    public static double RandomNormal()
    {
         double u, v, S;
     
         do
         {
             u = 2.0 * UnityEngine.Random.value - 1.0;
             v = 2.0 * UnityEngine.Random.value - 1.0;
             S = u * u + v * v;
         }
         while (S >= 1.0);
     
         // Standard Normal Distribution
         double z = u * Math.Sqrt(-2.0 * Math.Log(S) / S);
     
         return z;

    }
    // Update is called once per frame
    void Update()
    {
    	if(time==0){
    		InitParams();
    	}
    	h[0]=H0*Math.Sin(omega*time-kappa*x0)*(0.1+pulse);
  	hdot[0]=H0*omega*Math.Cos(omega*time-kappa*x0)*(0.1+pulse)+RandomNormal()*noise;
  	double time0=time;
	while(time-time0<Time.deltaTime*tscale){
	        integrate(time,timestep);
	        time+=timestep;
	}
	Vector3 pos0=transform.position;
        transform.position=((float)x)*transform.forward+(float)y*transform.right;
        U+=0.0001*((transform.position-pos0).magnitude-U*Time.deltaTime)-0.001*U*Time.deltaTime;
        transform.Rotate(0f,(float)Math.Asin((h[0]-h[h.Length-1])/l),0f,Space.World);
        double d=l/articulated_body.Length;
        int stride=h.Length/articulated_body.Length;
        for(int i=0; i<articulated_body.Length; ++i){
        	articulated_body[i].transform.position=transform.position-(float)(d*i-l/2)*transform.forward+(float)h[i*stride]*transform.right;
        }
        for(int i=1; i<articulated_body.Length; ++i){
        	Vector3 tail=articulated_body[i-1].transform.position-articulated_body[i-1].transform.forward*(float)d;
        	Vector3 tail2=articulated_body[i].transform.position-articulated_body[i].transform.forward*(float)d;
              	Vector3 u=(tail-tail2);
        	articulated_body[i].transform.forward=u.normalized;
        }
    }
}
