using System;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;
using Random = UnityEngine.Random;

/// <summary>
/// class that defines the reward distribution (orientation, reward)
/// </summary>
/// <remarks>
///   <para>
///     TODO: describe this class in detail
///   </para>
/// </remarks> 
public class RewardTensor 
{
    public static float[] scaleByMat(float[][] M, float[] x)
    {
        float[] y=new float[9];
        for(int i=0; i<9; ++i){
            y[i]=0f;
            for(int k=0; k<9; ++k){
                y[k]+=M[i][k]*y[k];
            }
        }
	Debug.Log("y= "+String.Join(",",y.Select(p=>p.ToString()).ToArray()));
        return y;
    }
    public static float maxR(float[] x, out Vector3 best)
    {
        float dmax=-1000000f;
        best=Vector3.zero;
        Vector3[] dirs={Vector3.up,Vector3.left,Vector3.right,Vector3.down, Vector3.back, Vector3.forward};
        for(int i=0; i<6; ++i){
            float d=Utils.sh_dot(x,dirs[i]);
                if(d>dmax){
                    dmax=d;
                best=dirs[i];
            }
        }
        for(int i=0; i<100; ++i){
            Vector3 v=Random.onUnitSphere;
            float d=Utils.sh_dot(x,v);
            if(d>dmax){
                dmax=d;
                best=v;
            }
        }
		return Utils.sh_dot(x,best);
	}

    public static float minR(float[] x)
	{
	    float d=100000000f;
	    Vector3[] dirs={Vector3.up,Vector3.left,Vector3.right,Vector3.down, Vector3.back, Vector3.forward};
	    for(int i=0; i<6; ++i){
		d=Mathf.Min(Utils.sh_dot(x,dirs[i]),d);
	    }
	    for(int i=0; i<100; ++i){
		d=Mathf.Min(Utils.sh_dot(x,Random.onUnitSphere),d);
	    }
	    return d;
	}

    public float[][] V;

    /// <summary>
    ///   Writes the V tensor to file for use with future simulations
    /// </summary>
    public void savetoFile(string fname = "mem")
    {
	Debug.Log("saving reward V matrix to FishMemory/.bin...");
	using (BinaryWriter writer=new BinaryWriter(File.Open("Assets/FishMemory/"+fname+".bin", FileMode.OpenOrCreate)))
	{
	    for(int i=0; i < V.Length; i++)
	    {
		for(int j=0; j < 9; j++)
		{
		    writer.Write(V[i][j]);
		}
	    }
	}
    }

    public void readFromFile(string fname="mem.bin")
    {
	if(File.Exists(fname))
	{
	    using(BinaryReader reader = new BinaryReader(File.Open("Assets/FishMemory/"+fname, FileMode.Open)))
	    {
		for(int i=0; i < nbins; i++)
		{
		    for(int j=0; j < 9; j++)
		    {
			V[i][j] = reader.ReadSingle();
		    }
		}
	    }
	}
    }

    int nbins=0;

    float maxd=0.0f;
    
    public Vector3 lastpos=new Vector3(0,1,0);

    public RewardTensor(int n_reward_bins, float max_dist=20f)
	{
		    maxd=max_dist;
		    nbins=n_reward_bins;
		    V=new float[nbins*nbins*nbins*nbins*nbins*nbins][];
		    for(int i=0; i<nbins*nbins*nbins*nbins*nbins*nbins; ++i){
                V[i]=new float[9];
                for(int j=0; j<9; ++j) V[i][j]=Random.value*-0.00001f;
            }
	}
    
    public void spot(Vector3 p)
    {
        lastpos=p;
    }

    public void degrade(float f, Vector3 p)
	{
            foreach(var v in V){
                v[0]-=0.0001f;
            }
	}
    
    public int bin(Vector3 q, float v, float w)
    {
	Vector3 p0=Vector3.up*maxd;
	Vector3 p1=Vector3.left*maxd;
	Vector3 p2=Vector3.back*maxd;
	Vector3 p3=lastpos;
	float px=Mathf.Min((q-p0).magnitude,maxd);
	float py=Mathf.Min((q-p1).magnitude,maxd);
	float pz=Mathf.Min((q-p2).magnitude,maxd);
	float pw=Mathf.Min((q-p3).magnitude,maxd);
	int ix=(int)(px*nbins/maxd);
	ix=(int) Mathf.Min(ix,nbins-1);
	int iy=(int)(py*nbins/maxd);
	iy=(int) Mathf.Min(iy,nbins-1);
	int iz=(int)(pz*nbins/maxd);
	iz=(int) Mathf.Min(iz,nbins-1);
	int iw=(int)(pw*nbins/maxd);
	iw=(int) Mathf.Min(iw,nbins-1);
	int ih=(int) (v*(nbins-1)*0.5f+0.5f*(nbins-1));
	int ik=(int) (w*(nbins-1)*0.5f+0.5f*(nbins-1));
	return ih*nbins*nbins*nbins*nbins*nbins+ih*nbins*nbins*nbins*nbins+ix*nbins*nbins*nbins+iy*nbins*nbins+iz*nbins+iw;
    }
    
    public void upd(Vector3 d, float R, Vector3 p, float v, float w)
    {
	String s="";
        
	float[] r=new float[]{R,R,R,R,R,R,R,R,R};
	float[] u1=Utils.sh_proj(r,d);
	for(int j=1; j<9; ++j){
	    V[bin(p,v,w)][j]+=u1[j];
	}
        
	for(int j=1; j<9; ++j){
	    s+=V[bin(p,v,w)][j]+" ";
	}
	Debug.Log(s);
    }
    
    public void scale(float[][] M, Vector3 p, float v, float w)
    {
	V[bin(p,v,w)]=scaleByMat(M,V[bin(p,v,w)]);
    }
    
    public float max(Vector3 p)
    {
	Vector3 tmp;
	float d=-999999999.0f;
	for(int i=0; i<nbins; ++i){
	    for(int j=0; j<nbins; ++j){
		d=Mathf.Max(d,maxR(V[bin(p,2f*i/(nbins-1f)-1f,2f*j/(nbins-1f)-1f)],out tmp));
	    }
	}
	return d;
    }
    
	public float min(Vector3 p)
	{
        float d=999999999.0f;
        for(int i=0; i<nbins; ++i){
            for(int j=0; j<nbins; ++j){
                d=Mathf.Min(d,minR(V[bin(p,2f*i/(nbins-1f)-1f,2f*j/(nbins-1f)-1f)]));
            }
        }
        return d;
	}
	public Vector3 opt(Vector3 d, ref float v, ref float w)
	{
		Vector3 best=Vector3.zero,tmp;
		float dmax=-999999999f;
        for(int i=0; i<nbins; ++i){
            for(int j=0; j<nbins; ++j){
                float d2=maxR(V[bin(d,i/(nbins-1f),j/(nbins-1f))],out tmp);
                if(d2>dmax){
                    v=2f*i/(nbins-1f)-1f;
                    w=2f*j/(nbins-1f)-1f;
                    dmax=d2;
                    best=tmp;
                }
            }
        }
		return best;
	}
	public void reset(Vector3 p, float v, float w)
	{
		for(int i=0; i<9; ++i) V[bin(p,v,w)][i]=0f;
	}
	public float eval(Vector3 p, Vector3 d, float v, float w)
	{
		return Utils.sh_dot(V[bin(p,v,w)],d);
	}
}
