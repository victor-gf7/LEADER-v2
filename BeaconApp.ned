package org.car2x.veins.modules.application.traci;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple BeaconApp extends DemoBaseApplLayer{
    @class(veins::BeaconApp);
    double windowTime = default(5.0);
}
