package fyp;

import java.awt.Canvas;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;

public class MethodAll extends Canvas implements MouseListener, Runnable{
	private static final long serialVersionUID = 1L;
    boolean begin,mouseClicked;
	Thread thread;
    int screenX=200;
    int screenY=200;
    int screen=200;
    int screenPosX=420;
    int screenPosY=40;
	int numLinks=3;		
    int cursorX=0; //mouse cursor x
    int cursorY=0; //mouse cursor y
	int goalMSetX=0;//cartesian goal point 
	int goalMSetY=0;//cartesian goal point
	int maxIterations=10000;// maximum iterations		
	Matrix goal;
			
	double stepScaleFactor=0.1; //set the m=10
	double epsilon = 0.01;
	//PINV
	int APINVcount=0;//begin iterations		
	double [] APINVlengths = new double[numLinks];
	Integer [] APINVx = new Integer[4];//original java paint 
	Integer [] APINVy = new Integer[4];//original java paint 	
    double [] APINVthetas = new double[numLinks]; //default radians		
    Matrix [] APINVjointLocations = new Matrix[numLinks]; 
    Matrix APINVjacobian, APINVjInverse, APINVendPoint, APINVdX, APINVdTheta;
	
	//CCD
	int ACCDcount=0;//begin iterations		
	double [] ACCDlengths = new double[numLinks];
	Integer [] ACCDx = new Integer[4];//original java paint 
	Integer [] ACCDy = new Integer[4];//original java paint 	
    double [] ACCDthetas = new double[numLinks]; //default radians		
    Matrix [] ACCDjointLocations = new Matrix[numLinks]; 
    Matrix  ACCDendPoint;

	//Secondary
	Matrix identity3;
	Integer [] ASECONDARYx = new Integer[4];//original java paint 
	Integer [] ASECONDARYy = new Integer[4];//original java paint   
    int ASECONDARYcount=0;//begin iterations	
    double [] ASECONDARYlengths = new double[numLinks];
    double [] ASECONDARYthetas = new double[numLinks]; //default radians
    Matrix [] ASECONDARYjointLocations = new Matrix[numLinks]; 
	Matrix ASECONDARYjacobian, ASECONDARYjInverse, ASECONDARYendPoint, ASECONDARYdX
	,ASECONDARYdTheta, dtheta2, ASECONDARYjplusj,ASECONDARYiminusJplusjDtheta,ASECONDARYdThetaInput;		
	
	//DLS
	double lamda=10;   
	Matrix identity2;
	Integer [] ADLSx = new Integer[4];//original java paint 
	Integer [] ADLSy = new Integer[4];//original java paint 			
    int ADLScount=0;//begin iteration
	double [] ADLSlengths = new double[numLinks];
	double [] ADLSthetas = new double[numLinks]; //default radians
	Matrix [] ADLSjointLocations = new Matrix[numLinks]; 
	Matrix ADLSjacobian, ADLSjInverse, ADLSendPoint, ADLSdX, ADLSdTheta, ADLSjJtpluslamdaSqIInverse, ADLSjJT;

	//JT
	Integer [] AJTx = new Integer[4];//original java paint 
	Integer [] AJTy = new Integer[4];//original java paint 
	int AJTcount=0;//begin iterations	
	double [] AJTlengths = new double[numLinks];
	double [] AJTthetas = new double[numLinks]; //default radians
	Matrix [] AJTjointLocations = new Matrix[numLinks]; 
	Matrix AJTjacobian, AJTjInverse, AJTendPoint, AJTdX,AJTdTheta, AJTjJT, AJTjJTDx, num,denom;
	double alpha = 0;
	
	public MethodAll(){  //setup the variables value	
		begin = true; 		
	    mouseClicked=false;
	    goal = new Matrix(2,1);
		this.setBackground (Color.white);
        this.setSize(screenX,screenY);
        this.setLocation(screenPosX,screenPosY);
        this.setVisible(true);
        this.addMouseListener(this);
        			
		//PINV
	    APINVendPoint= new Matrix(2,1);			
	    APINVdX= new Matrix(2,1);
	    APINVdTheta= new Matrix(numLinks,1);
	    APINVjacobian= new Matrix(2,numLinks);
	    APINVjInverse= new Matrix(numLinks,2);
	    
	    for(int i=0; i<numLinks; i++) {
	        APINVjointLocations[i]= new Matrix(2,1);
	        APINVjointLocations[i].elements[0][0]= 20+20*i;
	        APINVjointLocations[i].elements[1][0]= 0;
	        APINVthetas[i]= 0;
	        APINVlengths[i]= 20;  
	    }
		
		//CCD
		ACCDendPoint= new Matrix(2,1); 	    
	    for(int i=0; i<numLinks; i++) {
	        ACCDjointLocations[i]= new Matrix(2,1);
	        ACCDjointLocations[i].elements[0][0]= 20+20*i;
	        ACCDjointLocations[i].elements[1][0]= 0;
	        ACCDthetas[i]= 0;
	        ACCDlengths[i]= 20;
	        
	    }

		//Secondary
		ASECONDARYendPoint= new Matrix(2,1);
    	ASECONDARYdX= new Matrix(2,1);
    	ASECONDARYdTheta= new Matrix(numLinks,1);
		dtheta2=new Matrix(numLinks,1);
    	identity3= new Matrix(3,3);
    	ASECONDARYjplusj=new Matrix(3,3);
    	ASECONDARYiminusJplusjDtheta=new Matrix(3,3);
    	ASECONDARYdThetaInput= new Matrix(numLinks,1);
    	    
    	ASECONDARYjacobian= new Matrix(2,numLinks);
    	ASECONDARYjInverse= new Matrix(numLinks,2);
    	    
    	for(int i=0;i<3;i++){
    	    identity3.elements[i][i]=1;
    	    ASECONDARYdThetaInput.elements[i][0]=0;
    	}

    	identity3.elements[0][1]=0;
    	identity3.elements[0][2]=0;
    	identity3.elements[1][0]=0;
    	identity3.elements[1][2]=0;
    	identity3.elements[2][0]=0;
    	identity3.elements[2][1]=0;
    	    
    	for(int i=0; i<numLinks; i++) {
    	    ASECONDARYjointLocations[i]= new Matrix(2,1);
    	    ASECONDARYjointLocations[i].elements[0][0]= 20+20*i;
    	    ASECONDARYjointLocations[i].elements[1][0]= 0;
    	    ASECONDARYthetas[i]= 0;
    	    ASECONDARYlengths[i]= 20;
    	}
		
		//DLS
		ADLSendPoint= new Matrix(2,1);
    	ADLSdX= new Matrix(2,1);
    	    
    	ADLSdTheta= new Matrix(numLinks,1);
    	ADLSjJtpluslamdaSqIInverse=new Matrix(2,2);
    	ADLSjJT= new Matrix(2,2);
    	identity2= new Matrix(2,2);
    	    
    	ADLSjacobian= new Matrix(2,numLinks);
    	ADLSjInverse= new Matrix(numLinks,2);
    	    
    	for(int i=0; i<2;i++){
    	    identity2.elements[i][i]=1;
    	}
    	    
    	identity2.elements[0][1]=1;
    	identity2.elements[1][0]=1;
    	    
    	for(int i=0; i<numLinks; i++) {
    	    ADLSjointLocations[i]= new Matrix(2,1);
    	    ADLSjointLocations[i].elements[0][0]= 20+20*i;
    	    ADLSjointLocations[i].elements[1][0]= 0;
    	    ADLSthetas[i]= 0;
    	    ADLSlengths[i]= 20;
    	}
		
		//JT
		AJTendPoint= new Matrix(2,1);
    	AJTdX= new Matrix(2,1);
    	    
    	AJTdTheta= new Matrix(numLinks,1);

    	AJTjJT= new Matrix(2,2);
    	AJTjJTDx= new Matrix(2,1);
    	num=new Matrix(1,1);
    	denom= new Matrix(1,1);

    	AJTjacobian= new Matrix(2,numLinks);
    	AJTjInverse= new Matrix(numLinks,2);
    	            	    
    	for(int i=0; i<numLinks; i++) {
    	    AJTjointLocations[i]= new Matrix(2,1);
    	    AJTjointLocations[i].elements[0][0]= 20+20*i;
    	    AJTjointLocations[i].elements[1][0]= 0;
    	    AJTthetas[i]= 0;
    	    AJTlengths[i]= 20;
    	}
		  	
	}

	public void mouseClicked(MouseEvent e) {}
	public void mouseEntered(MouseEvent arg0) {}
	public void mouseExited(MouseEvent arg0) {}
	public void mouseReleased(MouseEvent arg0) {}
	public void mousePressed(MouseEvent e) {
		Rectangle bounds = bounds();
		mouseClicked=true;    				
		cursorX = e.getX();
		cursorY = e.getY();    		
		goalMSetX= e.getX()-bounds.width/2;
		goalMSetY= (bounds.height/2-e.getY());  		
        IK2DTopPanel.tfEEX.setText(goalMSetX+"");//cast to string
        IK2DTopPanel. tfEEY.setText(goalMSetY+"");//cast to string
		APINVcount=0;//set the count to 0    
		ACCDcount=0;//set the count to 0 
		ASECONDARYcount=0;//set the count to 0 
		ADLScount=0;//set the count to 0 
		AJTcount=0;//set the count to 0 
		Matrix.setGoal(goal,goalMSetX, goalMSetY);
		start();
        
	}
	
	public void start() {
        thread = new Thread(this);
        thread.setPriority(Thread.MIN_PRIORITY);
        thread.start();
	}

	public void stop() {
        if (thread != null)
          thread.interrupt();
        thread = null;
	}

	public void run() {
       Thread me = Thread.currentThread();
       while (thread == me) {
       	try {
				Thread.sleep(1000/5);
			} catch (InterruptedException e) {}
         repaint();
       }
       thread = null;
    }         
    
    public void condition() {
	
		//PINV
    	while (APINVcount<=maxIterations) {
    		if( (Math.sqrt(Math.abs(APINVendPoint.subtractFrom(goal).elements[0][0]))*2 
        			+(Math.abs(APINVendPoint.subtractFrom(goal).elements[1][0]))*2) <=1) {
    			 IK2DTopPanel.tfSuccess[0].setText("Yes");
    			stop();
    			break;
    		}// end if	
    		else{
    			IK2DTopPanel.tfSuccess[0].setText("No");
    			 IK2DTopPanel.tfCount[0].setText(APINVcount+""); 
      			APINVmoveToGoal_NLink(goal);	
      			if(APINVcount==maxIterations)
      				stop();
        	}
        }//end while
		
		//CCD
		while (ACCDcount<=maxIterations) {
    		if( (Math.sqrt(Math.abs(ACCDendPoint.subtractFrom(goal).elements[0][0]))*2 
        			+(Math.abs(ACCDendPoint.subtractFrom(goal).elements[1][0]))*2) <=1) {
    			 IK2DTopPanel.tfSuccess[1].setText("Yes");
    			stop();
    			break;
    		}// end if	
    		else{
    			 IK2DTopPanel.tfSuccess[1].setText("No");
    			 IK2DTopPanel.tfCount[1].setText(ACCDcount+""); 
      			ACCDmoveToGoal_NLink(goal); 	
      			if(ACCDcount==maxIterations)
      				stop();
        	}
        }//end while
		
		//secondary
		while (ASECONDARYcount<=maxIterations) {
    		if( (Math.sqrt(Math.abs(ASECONDARYendPoint.subtractFrom(goal).elements[0][0]))*2 
        			+(Math.abs(ASECONDARYendPoint.subtractFrom(goal).elements[1][0]))*2) <=1) {
    			 IK2DTopPanel.tfSuccess[2].setText("Yes");
    			stop();
    			break;
    		}// end if	
    		else{
    			 IK2DTopPanel.tfSuccess[2].setText("No");
    			 IK2DTopPanel.tfCount[2].setText(ASECONDARYcount+""); 
      			ASECONDARYmoveToGoal_NLink(goal);  	
      			if(ASECONDARYcount==maxIterations)
      				stop();
        	}
        }//end while
		
		//DLS
		while (ADLScount<=maxIterations) {
    		if( (Math.sqrt(Math.abs(ADLSendPoint.subtractFrom(goal).elements[0][0]))*2 
        			+(Math.abs(ADLSendPoint.subtractFrom(goal).elements[1][0]))*2) <=1) {
    			 IK2DTopPanel.tfSuccess[3].setText("Yes");
    			stop();
    			break;
    		}// end if	
    		else{
    			 IK2DTopPanel.tfSuccess[3].setText("No");
    			 IK2DTopPanel.tfCount[3].setText(ADLScount+""); 
      			ADLSmoveToGoal_NLink(goal);
      			//animate(5);   	
      			if(ADLScount==maxIterations)
      				stop();
        	}
        }//end while
		
		//JT
		while (AJTcount<=maxIterations) {
    		if( (Math.sqrt(Math.abs(AJTendPoint.subtractFrom(goal).elements[0][0]))*2 
        			+(Math.abs(AJTendPoint.subtractFrom(goal).elements[1][0]))*2) <=1) {
    			 IK2DTopPanel.tfSuccess[4].setText("Yes");
        		break;
        	}// end if	
        	else{
        		 IK2DTopPanel.tfSuccess[4].setText("No");
        		 IK2DTopPanel.tfCount[4].setText(AJTcount+"");      			
          		AJTmoveToGoal_NLink(goal);        			
            }
        }//end while
    }//end condition
           
    public void APINVmoveToGoal_NLink(Matrix gl) {
    	++APINVcount;
    	double sum= 0;
        double [] ct= new double[numLinks];
        double [] st= new double[numLinks];      
        for(int i=0; i<numLinks; i++) {
          sum+= APINVthetas[i];
          ct[i]= Math.cos(sum);
          st[i]= Math.sin(sum);
        }           
        APINVjointLocations[0].elements[0][0]= APINVlengths[0]*ct[0];
        APINVjointLocations[0].elements[1][0]= APINVlengths[0]*st[0];          

        for(int i=1; i<numLinks; i++) {     	
        	APINVjointLocations[i].elements[0][0]= APINVjointLocations[i-1].elements[0][0] + APINVlengths[i]*ct[i];
        	APINVjointLocations[i].elements[1][0]= APINVjointLocations[i-1].elements[1][0] + APINVlengths[i]*st[i];
        }

        APINVendPoint= APINVjointLocations[numLinks-1];// last point
        APINVdX= gl.subtractFrom(APINVendPoint).multiply(stepScaleFactor);

        for(int i=0; i<numLinks; i++) {
        	APINVjacobian.elements[0][i]= APINVjacobian.elements[1][i]= 0;         
        	for(int j= i; j<numLinks; j++) {
        		APINVjacobian.elements[0][i] += -APINVlengths[j]*st[j];
        		APINVjacobian.elements[1][i] += APINVlengths[j]*ct[j];           
        	}     	
        }

        APINVjInverse= APINVjacobian.pseudoInverse();  
        APINVdTheta= APINVjInverse.multiply(APINVdX);
        for(int i=0; i<numLinks; i++) {
          APINVthetas[i]+=APINVdTheta.elements[i][0];
        }
    } 
      
	public void ACCDmoveToGoal_NLink(Matrix gl) {
		++ACCDcount;
    	double sum= 0;
        double [] ct= new double[numLinks];
        double [] st= new double[numLinks];      
        for(int i=0; i<numLinks; i++) {
          sum+= ACCDthetas[i];
          ct[i]= Math.cos(sum);
          st[i]= Math.sin(sum);
        } 
        
        ACCDjointLocations[0].elements[0][0]= ACCDlengths[0]*ct[0];
        ACCDjointLocations[0].elements[1][0]= ACCDlengths[0]*st[0];          

        for(int i=1; i<numLinks; i++) {     	
        	ACCDjointLocations[i].elements[0][0]= ACCDjointLocations[i-1].elements[0][0] + ACCDlengths[i]*ct[i];
        	ACCDjointLocations[i].elements[1][0]= ACCDjointLocations[i-1].elements[1][0] + ACCDlengths[i]*st[i];

        }

       ACCDendPoint= ACCDjointLocations[numLinks-1];// last point

		for( int boneIdx = numLinks-2; boneIdx >=0; boneIdx-- )
		{
			double curToEndX = ACCDendPoint.elements[0][0] - ACCDjointLocations[boneIdx].elements[0][0];
			double curToEndY = ACCDendPoint.elements[1][0] - ACCDjointLocations[boneIdx].elements[1][0];
			double curToEndMag = Math.sqrt( curToEndX*curToEndX + curToEndY*curToEndY );
			double curToTargetX = goal.elements[0][0] - ACCDjointLocations[boneIdx].elements[0][0];
			double curToTargetY = goal.elements[1][0] - ACCDjointLocations[boneIdx].elements[1][0];
			double curToTargetMag = Math.sqrt(   curToTargetX*curToTargetX + curToTargetY*curToTargetY );
			double cosRotAng;
			double sinRotAng;
			double endTargetMag = (curToEndMag*curToTargetMag);
			if( endTargetMag <= epsilon ){
				cosRotAng = 1;
				sinRotAng = 0;
			}
			else{
				cosRotAng = (curToEndX*curToTargetX + curToEndY*curToTargetY) / endTargetMag;
				sinRotAng = (curToEndX*curToTargetY - curToEndY*curToTargetX) / endTargetMag;
			}

			double rotAng = Math.acos( Math.max(-1, Math.min(1,cosRotAng) ) );
			if( sinRotAng < 0.0 )
				rotAng = -rotAng;
			
			ACCDendPoint.elements[0][0] = ACCDjointLocations[boneIdx].elements[0][0] + cosRotAng*curToEndX - sinRotAng*curToEndY;
			ACCDendPoint.elements[1][0] = ACCDjointLocations[boneIdx].elements[1][0] + sinRotAng*curToEndX + cosRotAng*curToEndY;
			ACCDthetas[boneIdx+1] = ListOfEquation.SimplifyAngle( ACCDthetas[boneIdx+1] + rotAng );
		}
		
		double curToEndX = ACCDendPoint.elements[0][0];
			double curToEndY = ACCDendPoint.elements[1][0];
			double curToEndMag = Math.sqrt( curToEndX*curToEndX + curToEndY*curToEndY );
			double curToTargetX = goal.elements[0][0];
			double curToTargetY = goal.elements[1][0];
			double curToTargetMag = Math.sqrt( curToTargetX*curToTargetX+ curToTargetY*curToTargetY );
			double cosRotAng;
			double sinRotAng;
			double endTargetMag = (curToEndMag*curToTargetMag);
			if( endTargetMag <= epsilon ){
				cosRotAng = 1;
				sinRotAng = 0;
			}
			else{
				cosRotAng = (curToEndX*curToTargetX + curToEndY*curToTargetY) / endTargetMag;
				sinRotAng = (curToEndX*curToTargetY - curToEndY*curToTargetX) / endTargetMag;
			}
			double rotAng = Math.acos( Math.max(-1, Math.min(1,cosRotAng) ) );
			if( sinRotAng < 0.0 )
				rotAng = -rotAng;
			ACCDendPoint.elements[0][0] = cosRotAng*curToEndX - sinRotAng*curToEndY;
			ACCDendPoint.elements[1][0] = sinRotAng*curToEndX + cosRotAng*curToEndY;
			ACCDthetas[0] = ListOfEquation.SimplifyAngle( ACCDthetas[0] + rotAng );
    }  
	
	public void ASECONDARYmoveToGoal_NLink(Matrix gl) {
		++ASECONDARYcount;
    	double sum= 0;
        double [] ct= new double[numLinks];
        double [] st= new double[numLinks];      
        for(int i=0; i<numLinks; i++) {
          sum+= ASECONDARYthetas[i];
          ct[i]= Math.cos(sum);
          st[i]= Math.sin(sum);
        }           
        ASECONDARYjointLocations[0].elements[0][0]= ASECONDARYlengths[0]*ct[0];
        ASECONDARYjointLocations[0].elements[1][0]= ASECONDARYlengths[0]*st[0];          

        for(int i=1; i<numLinks; i++) {     	
        	ASECONDARYjointLocations[i].elements[0][0]= ASECONDARYjointLocations[i-1].elements[0][0] + ASECONDARYlengths[i]*ct[i];
        	ASECONDARYjointLocations[i].elements[1][0]= ASECONDARYjointLocations[i-1].elements[1][0] + ASECONDARYlengths[i]*st[i];

        }

		ASECONDARYendPoint= ASECONDARYjointLocations[numLinks-1];// last point
		ASECONDARYdX= gl.subtractFrom(ASECONDARYendPoint).multiply(stepScaleFactor);
            
        for(int i=0; i<numLinks; i++) {
          	ASECONDARYjacobian.elements[0][i]= ASECONDARYjacobian.elements[1][i]= 0;           
          	for(int j= i; j<numLinks; j++) {
				ASECONDARYjacobian.elements[0][i] += -ASECONDARYlengths[j]*st[j];
            	ASECONDARYjacobian.elements[1][i] += ASECONDARYlengths[j]*ct[j];               
            	}               	
        }
    
		ASECONDARYjInverse= ASECONDARYjacobian.pseudoInverse();  
		ASECONDARYdTheta= ASECONDARYjInverse.multiply(ASECONDARYdX);
		ASECONDARYjplusj= ASECONDARYjInverse.multiply(ASECONDARYjacobian);
		ASECONDARYiminusJplusjDtheta= identity3.subtractFrom(ASECONDARYjplusj).multiply(ASECONDARYdThetaInput);
		ASECONDARYdThetaInput= ASECONDARYdTheta.plusFrom(ASECONDARYiminusJplusjDtheta);

        for(int i=0; i<numLinks; i++) {
             ASECONDARYthetas[i]+=ASECONDARYdThetaInput.elements[i][0];
        }              
            
    }  
	
	 public void ADLSmoveToGoal_NLink(Matrix gl) {
		 ++ADLScount;
    	double sum= 0;
        double [] ct= new double[numLinks];
        double [] st= new double[numLinks];      
        for(int i=0; i<numLinks; i++) {
          sum+= ADLSthetas[i];
          ct[i]= Math.cos(sum);
          st[i]= Math.sin(sum);
        }           
        ADLSjointLocations[0].elements[0][0]= ADLSlengths[0]*ct[0];
        ADLSjointLocations[0].elements[1][0]= ADLSlengths[0]*st[0];          

        for(int i=1; i<numLinks; i++) {     	
        	ADLSjointLocations[i].elements[0][0]= ADLSjointLocations[i-1].elements[0][0] + ADLSlengths[i]*ct[i];
        	ADLSjointLocations[i].elements[1][0]= ADLSjointLocations[i-1].elements[1][0] + ADLSlengths[i]*st[i];

        }

		ADLSendPoint= ADLSjointLocations[numLinks-1];// last point
        ADLSdX= gl.subtractFrom(ADLSendPoint).multiply(stepScaleFactor);              
        for(int i=0; i<numLinks; i++) {
           	ADLSjacobian.elements[0][i]= ADLSjacobian.elements[1][i]= 0;
           	for(int j= i; j<numLinks; j++) {
           		ADLSjacobian.elements[0][i] += -ADLSlengths[j]*st[j];
           		ADLSjacobian.elements[1][i] += ADLSlengths[j]*ct[j];               
           	}
        }
            
        ADLSjInverse= ADLSjacobian.pseudoInverse();  
		ADLSjJT= ADLSjacobian.multiply(ADLSjacobian.transpose());
		ADLSjJtpluslamdaSqIInverse= ADLSjJT.plusFrom(identity2.multiply(lamda*lamda)).pseudoInverse();
		ADLSdTheta= ADLSjacobian.transpose().multiply(ADLSjJtpluslamdaSqIInverse).multiply(ADLSdX);
        for(int i=0; i<numLinks; i++) {
            ADLSthetas[i]+=ADLSdTheta.elements[i][0];
        }                     
    }  
	
	 public void AJTmoveToGoal_NLink(Matrix gl) {
		 	++AJTcount;//increment
        	double sum= 0;
            double [] ct= new double[numLinks];
            double [] st= new double[numLinks];       
            for(int i=0; i<numLinks; i++) {
              sum+= AJTthetas[i];
              ct[i]= Math.cos(sum);
              st[i]= Math.sin(sum);
            }
            
            AJTjointLocations[0].elements[0][0]= AJTlengths[0]*ct[0];
            AJTjointLocations[0].elements[1][0]= AJTlengths[0]*st[0];
            
            for(int i=1; i<numLinks; i++) {     	
            	AJTjointLocations[i].elements[0][0]= AJTjointLocations[i-1].elements[0][0] + AJTlengths[i]*ct[i];
            	AJTjointLocations[i].elements[1][0]= AJTjointLocations[i-1].elements[1][0] + AJTlengths[i]*st[i];
        
            }

            AJTendPoint= AJTjointLocations[numLinks-1];// last point
            AJTdX= gl.subtractFrom(AJTendPoint).multiply(stepScaleFactor);
            for(int i=0; i<numLinks; i++) {
            	AJTjacobian.elements[0][i]= AJTjacobian.elements[1][i]= 0;
           
            	for(int j= i; j<numLinks; j++) {
            		AJTjacobian.elements[0][i] += -AJTlengths[j]*st[j];
            		AJTjacobian.elements[1][i] += AJTlengths[j]*ct[j];               
            	}
            	
            }
            
            AJTjInverse= AJTjacobian.pseudoInverse();  
            AJTjJT= AJTjacobian.multiply(AJTjacobian.transpose());              
    	    AJTjJTDx= AJTjJT.multiply(AJTdX);
    	    num= AJTdX.transpose().multiply(AJTjJTDx);
    	    denom= AJTjJTDx.transpose().multiply(AJTjJTDx);
    	    
    	    alpha=num.elements[0][0]/denom.elements[0][0];
    	    AJTdTheta=AJTjacobian.transpose().multiply(alpha).multiply(AJTdX);

            for(int i=0; i<numLinks; i++) {
             AJTthetas[i]+=AJTdTheta.elements[i][0];
            } 
        }  
	
	public void paint(Graphics g){
		super.paint(g);   		
		Graphics2D g2 = (Graphics2D) g;
		draw2D.displayString(g,"Serif",12,"All",5, 180);
		
		for(int i=0; i<8;i++){
			for (int j =0 ;j<8;j++){
    			g2.draw(new Rectangle2D.Double(60+j*10,60+i*10,10,10));
    		}
		}
    	
		g2.setPaint(Color.red);
		draw2D.displayString(g,"Serif",10,"PINV: red",5, 15);   
		g2.setPaint(Color.cyan);
		draw2D.displayString(g,"Serif",10,"CCD: cyan",5, 30);
		g2.setPaint(Color.green);
		draw2D.displayString(g,"Serif",10,"Secondary: green",5, 45);
		g2.setPaint(Color.gray);
		draw2D.displayString(g,"Serif",10,"DLS: gray",5, 60);
		g2.setPaint(Color.blue);
		draw2D.displayString(g,"Serif",10,"JT: blue",5, 75);
		
        Ellipse2D.Double circle2 = new Ellipse2D.Double(40, 40, 120, 120);
        g2.setPaint(Color.black);
        g2.draw(circle2);
        draw2D.displayString(g,"Serif",8,"1  2   3   4  17 18 19 20 ",65, 68);
        draw2D.displayString(g,"Serif",8,"5  6   7   8  21 22 23 24 ",65, 78);
        draw2D.displayString(g,"Serif",8,"9 10 11 12 25 26 27 28 ",65, 88);
        draw2D.displayString(g,"Serif",8,"13 14 15 16 29 30 31 32 ",62, 98);
        draw2D.displayString(g,"Serif",8,"33 34 35 36 49 50 51 52 ",62, 108);
        draw2D.displayString(g,"Serif",8,"37 38 39 40 53 54 55 56 ",62, 118);
        draw2D.displayString(g,"Serif",8,"41 42 43 44 57 58 59 60 ",62, 128);
        draw2D.displayString(g,"Serif",8,"45 46 47 48 61 62 63 64 ",62, 138);
   
        
		APINVx[0] =screen/2;	//default center point
	    APINVy[0] =screen/2; //default center point
		ACCDx[0] =screen/2;	//default center point
	    ACCDy[0] =screen/2; //default center point        
		ASECONDARYx[0] =screen/2;	//default center point
	    ASECONDARYy[0] =screen/2; //default center point
		ADLSx[0] =screen/2;	//default center point
	    ADLSy[0] =screen/2; //default center point
		AJTx[0] =screen/2;	//default center point
	    AJTy[0] =screen/2; //default center point
	        	       
		if(mouseClicked==true){		
			draw2D.drawCircle(g,cursorX,cursorY,Color.red);//draw the target
			condition();
		}	  
			
		for(int i=0; i<numLinks;i++){      //convert radians to degree
			 IK2DTopPanel.tfTheta[i].setText(String.format(""));//cast to string
		}   		 
		draw2D.originLine(g,screen);

		if(begin==true){
			for(int i=0; i<numLinks; i++) {
    	        APINVx[i+1] = (int)APINVjointLocations[i].elements[0][0]+screen/2;
    	        APINVy[i+1] = screen/2-(int)APINVjointLocations[i].elements[1][0];
    	        ACCDx[i+1] = (int)ACCDjointLocations[i].elements[0][0]+screen/2;
    	        ACCDy[i+1] = screen/2-(int)ACCDjointLocations[i].elements[1][0];
			    ASECONDARYx[i+1] = (int)ASECONDARYjointLocations[i].elements[0][0]+screen/2;
     	        ASECONDARYy[i+1] = screen/2-(int)ASECONDARYjointLocations[i].elements[1][0];
    	        ADLSx[i+1] = (int)ADLSjointLocations[i].elements[0][0]+screen/2;
    	        ADLSy[i+1] = screen/2-(int)ADLSjointLocations[i].elements[1][0];
        	    AJTx[i+1] = (int)AJTjointLocations[i].elements[0][0]+screen/2;
        	    AJTy[i+1] = screen/2-(int)AJTjointLocations[i].elements[1][0];
    	    }    			
		}
		
		draw2D.displayLineWithColor(g,APINVx, APINVy,Color.red);			
		draw2D.displayLineWithColor(g,ACCDx, ACCDy,Color.cyan);
		draw2D.displayLineWithColor(g,ASECONDARYx, ASECONDARYy,Color.green);
		draw2D.displayLineWithColor(g,ADLSx, ADLSy,Color.gray);
		draw2D.displayLineWithColor(g,AJTx, AJTy,Color.blue);
		
        for(int i=0; i<=numLinks;i++){// draw the all the joint node
        	draw2D.drawCircle(g, APINVx[i], APINVy[i],Color.red); //joint1
        	draw2D.drawCircle(g, ACCDx[i], ACCDy[i], Color.cyan); //joint1
        	draw2D.drawCircle(g, ASECONDARYx[i], ASECONDARYy[i],Color.green); //joint1
        	draw2D.drawCircle(g, ADLSx[i], ADLSy[i],Color.gray); //joint1
        	draw2D.drawCircle(g, AJTx[i], AJTy[i],Color.blue); //joint
		}					       	 
	}
}