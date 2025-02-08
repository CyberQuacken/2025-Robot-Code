package frc.robot.Objects;

public class Vector {
    double x;
    double y;

    public void Vector(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double X (){
        return x;
    }

    public double Y (){
        return y;
    }

    @Override
    public String toString(){
        return "( " + x +", " + y +" )";
    }
}
