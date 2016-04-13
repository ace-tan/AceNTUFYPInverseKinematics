package fyp;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;

public class draw2D {
	
    public static void displayString(Graphics g,String stringType, int fontSize,String stringContent, int x, int y){
        Graphics2D g2d = (Graphics2D)g;
        //set the Font properties by type, pattern, size 
        Font font = new Font(stringType, Font.PLAIN, fontSize);
        g2d.setFont(font); 
        g2d.drawString(stringContent, x, y); 
    }//end displayString
    
    //mid lines
    public static void originLine(Graphics g, int screen) {
        Graphics2D g2 = (Graphics2D) g;
        g2.setPaint(Color.black);
        g2.drawLine(screen/2, 0, screen/2, 300);//vertical line
        g2.drawLine(0, screen/2, 300, screen/2);//horizontal line
    } 
    
    //display joint
    public static void drawCircle(Graphics g, double xDbl, double yDbl,Color colorVar) {
        Graphics2D g2 = (Graphics2D) g;
        Ellipse2D.Double circle = new Ellipse2D.Double(xDbl, yDbl, 5, 5);
        g2.setPaint(colorVar);
        g2.draw(circle);
        g2.fill(circle);
    } 
    
    //display joint
    public static void drawSquare(Graphics g, double xDbl, double yDbl,Color colorVar) {
        Graphics2D g2 = (Graphics2D) g;
        Rectangle2D.Double Square = new Rectangle2D.Double(xDbl, yDbl, 5, 5);
        g2.setPaint(colorVar);
        g2.draw(Square);
        g2.fill(Square);
    } 
    
    public static void displayLine(Graphics g, Integer [] x,Integer [] y) {
        Graphics2D g2 = (Graphics2D) g;
        //GeneralPath p = new GeneralPath(GeneralPath.WIND_NON_ZERO);
        //create and set the stroke
        g2.setStroke(new BasicStroke(3.0f));
        g2.setPaint(Color.CYAN);        //set the paint color
        for(int i=0; i<3 ; i++){
            g2.drawLine(x[i]+1, y[i]+1,x[i+1]+1, y[i+1]+1);
        }
    }
    
    public static void displayLineWithColor(Graphics g, Integer [] x,Integer [] y,Color newColor) {
        Graphics2D g2 = (Graphics2D) g;
        //GeneralPath p = new GeneralPath(GeneralPath.WIND_NON_ZERO);
        //create and set the stroke
        g2.setStroke(new BasicStroke(3.0f));
        g2.setPaint(newColor);        //set the paint color
        for(int i=0; i<3 ; i++){
            g2.drawLine(x[i]+1, y[i]+1,x[i+1]+1, y[i+1]+1);
        }
    }
       
    public static void displayNode(Graphics g, Integer [] x,Integer [] y,Color [] newColor,int numLinks) {
        for(int i=0; i<=numLinks;i++){// draw the all the joint node
        	drawCircle(g, x[i], y[i],newColor[i]); //joint1
        }
    }
       
    public static void displayNodeSq(Graphics g, Integer [] x,Integer [] y,Color [] newColor,int numLinks) {
        for(int i=0; i<=numLinks;i++){// draw the all the joint node
        	drawSquare(g, x[i], y[i],newColor[i]); //joint1
        }
    }
}
