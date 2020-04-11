function sysCall_init()
    -- Initiate Simulation Time
    sim.setFloatSignal('Turtlebot2_simulation_time', 0)

    -- Simulation
    remoteAPI = sim.getScriptSimulationParameter(sim.handle_self,'remoteAPIEnabled')
    if(remoteAPI) then
        simRemoteApi.start(19999, 1300, false, false)
    end
end

function sysCall_sensing()
        -- SIMULATION TIME --
        simTime = sim.getSimulationTime()
        sim.setFloatSignal('Turtlebot2_simulation_time', simTime)
end

function sysCall_cleanup() 
    if(remoteAPI) then
        simRemoteApi.stop(19999) 
    end
end