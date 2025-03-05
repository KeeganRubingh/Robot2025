<h1>Anode</h1>
<p>Anode is an easy-to-use tuning tool to streamline FRC teams fiddling with values during competitions. It uses annotations and reflection to define and manage variables which can be tuned through SmartDashboard.</p>
<h2>Implementation</h2>    
<ol>
<li>Add the <code>@AnodeObject(Key = String)</code> annotation to an object you want to be tunable (usually your subsystems)</li>
<li>Add a <code>String</code> field in your object, and annotate it with <code>@AnodeInstanceName()</code>. This should be unique for any given instance of your object, but I suggest making it human readable.</li>
<li>In your object's constructor (or any time after) call <code>AnodeManager.getInstance().addObjects(Object yourObject)</code>. This will bind the anode manager to your object.</li>
<li>Add the <code>@AnodeTunableParameter(Key = String)</code> annotation to any <code>Double</code> variables you want to be able to tune.</li>
<li>When you want to update your tuning variables, call <code>AnodeManager.getInstance().updateObject(Object objectToUpdate)</code> or <code>AnodeManager.getInstance().updateAll()</code>. This will set all annotated doubles to their values in SmartDashboard</li>
</ol> 
