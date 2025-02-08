package frc.robot.Objects;

public class Vector {
    double x;
    double y;

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector(){
        x = 0;
        y = 0;
    }

    public double X (){
        return x;
    }

    public double Y (){
        return y;
    }

    public void setX(double x){
        this.x = x;
    }

    public void setY (double y){
        this.y = y;
    }

    @Override
    public String toString(){
        return "( " + x +", " + y +" )";
    }
}
