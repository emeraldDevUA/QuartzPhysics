package org.tourmaline.PlanePhysics;


import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import static java.lang.StringTemplate.STR;

@AllArgsConstructor
@NoArgsConstructor
@Setter
@Getter
public class Tuple<A, B>{
    public  A a;
    public  B b;

    @Override
    public String toString(){
        return STR."<\{a} \{b}>";
    }
}
