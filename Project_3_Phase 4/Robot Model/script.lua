function sysCall_init()
    -- Simulation
    remoteAPI = sim.getScriptSimulationParameter(sim.handle_self,'remoteAPIEnabled')
    if(remoteAPI) then
        simRemoteApi.start(19999, 1300, false, false)
    end
end

function sysCall_cleanup() 
    if(remoteAPI) then
        simRemoteApi.stop(19999) 
    end
end