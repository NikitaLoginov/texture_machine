#pragma once

#include "daisy_seed.h"
#include "knobs.h"
#include "pads.h"
#include "switches.h"
#include "mvalue.h"
#include "latch.h"
#include "log.h"

namespace synthux {

/**
 * Unified hardware wrapper for Bends.
 * Bundles the 12 capacitive pads with continuous pressure, 8 analog inputs (knobs/faders),
 * and 2 toggle switches.
 */
class Touch {
public:
    void Init(daisy::DaisySeed& hw) {
        _knobs.Init(hw);
        _pads.Init(hw);
        _switches.Init();
        hw.adc.Start();
    }

    void Process() {
        _pads.Process();
        _knobs.Process();
    }

    Knobs& knobs() { return _knobs; }
    Pads& pads() { return _pads; }
    Switches& switches() { return _switches; }

private:
    Knobs _knobs;
    Pads _pads;
    Switches _switches;
};

} // namespace synthux
