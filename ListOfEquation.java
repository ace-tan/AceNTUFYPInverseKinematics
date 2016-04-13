package fyp;

//class
public class ListOfEquation {
	
	public static double convertRadtoDegree(double radians){
		double degree;
		degree = radians*180 / Math.PI;
		return degree;
		
	}
		
	public static double SimplifyAngle(double angle){
		angle = angle % (2.0 * Math.PI);
		if( angle < -Math.PI )
			angle += (2.0 * Math.PI);
		else if( angle > Math.PI )
			angle -= (2.0 * Math.PI);
		return angle;
	}
	   
	public static Double [] Calct(Double[] ct,int numLinks, Double [] thetas){
    	double sum= 0;
	    for(int i=0; i<numLinks; i++) {
	        sum+= thetas[i];
	        ct[i]= Math.cos(sum);
	    }
		return ct;
	}
	
	public static Double [] Calst(Double[] st,int numLinks, Double [] thetas){
    	double sum= 0;
	    for(int i=0; i<numLinks; i++) {
	        sum+= thetas[i];
	        st[i]= Math.sin(sum);
	    }
		return st;
	}
	  
	public static Double [] CalTheta(int numLinks, Double [] thetas, Matrix dTheta){
	    // increase theta by dTheta
	    for(int i=0; i<numLinks; i++) {
	      thetas[i]+=dTheta.elements[i][0];
	    }
		return thetas;
	}
}//end class
