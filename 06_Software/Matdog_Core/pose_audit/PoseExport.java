import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.program.model.listing.*;
import java.io.*;
public class PoseExport extends GhidraScript {
 public void run() throws Exception {
  File dir=new File(getScriptArgs()[0]);dir.mkdirs();
  DecompInterface decomp=new DecompInterface();decomp.openProgram(currentProgram);
  FunctionIterator it=currentProgram.getFunctionManager().getFunctions(true);
  while(it.hasNext()) {
   Function f=it.next();long a=f.getEntryPoint().getOffset();
   if(a<0x400d0020L||a>=0x400e0000L)continue;
   try(PrintWriter w=new PrintWriter(new File(dir,f.getEntryPoint()+".txt"))) {
    w.println(f.getEntryPoint()+" "+f.getName());
    DecompileResults r=decomp.decompileFunction(f,30,monitor);
    if(r.decompileCompleted())w.println(r.getDecompiledFunction().getC());else w.println(r.getErrorMessage());
    InstructionIterator ins=currentProgram.getListing().getInstructions(f.getBody(),true);
    while(ins.hasNext()){Instruction i=ins.next();w.println(i.getAddress()+" "+i.toString());}
   }
  }
  decomp.dispose();
 }
}
