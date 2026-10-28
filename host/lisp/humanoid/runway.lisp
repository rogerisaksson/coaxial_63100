;; =============================================================================
;; 1. DRY SIGNAL: NOMINAL BIOMECHANICAL BASELINE (DRYWALK)
;; =============================================================================
(defstate nominal-biped-gait
  (:clock
    :bpm 104.0
    :swing-gate 0.12          ; Natural double-support asymmetry
    :phase-jitter-ms 14.5)    ; Biological variance (humanize)

  (:kinematics
    :calcaneus      (:attack-ms 35.0 :decay-ms 120.0 :sustain 0.40) ; Soft heel-to-toe roll
    :tibiofemoral   (:flexion-cushion-rad 0.26 :compliance 0.85)    ; Knee joint absorbs transient
    :tarsal-coronal (:track-width-mm 115.0 :pan-spread 0.35)        ; Nominal step width (stereo)
    :sacroiliac-z   (:vertical-bounce-mm 22.0 :damping 0.707)       ; Critically damped CoM
    :iliac-coronal  (:lfo-amp-rad 0.06 :wave :triangle)             ; Moderate pelvic obliquity
    :l4-l5-lumbar   (:dc-offset-rad -0.15 :q-factor 0.40)           ; Neutral lordosis
    :c7-t12-thoracic(:dc-offset-rad 0.05 :cutoff-hz 8.5 :slope-db 6); Unconstrained torso
    :glenohumeral   (:phase-shift-rad 3.14159 :gain-db -3.0)        ; Natural contralateral arm swing
    :c1-c2-atlas    (:dc-offset-rad 0.0 :headroom-rad 0.15)         ; Neutral cervical alignment
    :craniofacial   (:velocity-sensitivity 0.80)))                  ; Nominal facial micro-motorics


;; =============================================================================
;; 2. WET INSERT: VULCANIZED-SOLE / STREETWEAR RUNWAY DSP MODIFIER
;; =============================================================================
(defmodifier sneaker-sidechain-fx [gait-stream]
  (-> gait-stream
      (quantize-grid
        :target-bpm 114.0
        :swing-gate 0.04          ; Laid-back MPC pocket on even steps
        :strip-jitter true)

      (shape-transients :target :calcaneus
        :saturation-mode :tape-warmth
        :attack-ms 9.5            ; EVA midsole compression window
        :decay-ms 65.0            ; Full plantar roll-off
        :sustain 0.25             ; Sub-bass floor grip
        :soft-clip-node :tibiofemoral
        :flexion-cushion-rad 0.09 ; Unlatched knee; streetwear bounce
        :compliance 0.35)

      (mid-side-rebalance
        :narrow-stereo (:node :tarsal-coronal :track-width-mm 42.0 :pan-spread 0.12)
        :stereo-mod    (:node :iliac-coronal  :wave :sine-warm :lfo-amp-rad 0.115 :width 0.55))

      (route-sidechain
        :key-input :calcaneus
        :target :sacroiliac-z
        :topology :opto-vca       ; Soft-knee optical compression
        :ratio 4.5
        :attack-ms 12.0
        :release-ms 280.0
        :release-curve :logarithmic
        :duck-depth-mm -26.0
        :makeup-elevation-mm 0.0)

      (apply-filter-bank
        (dc-bias         :target :l4-l5-lumbar    :offset-rad -0.05 :q-factor 0.30)
        (dc-bias         :target :c7-t12-thoracic :offset-rad 0.22  :q-factor 0.85) ; Kyphotic slouch
        (butterworth-lpf :target :c7-t12-thoracic :cutoff-hz 2.4    :slope-db 12)
        (phase-delay     :target :glenohumeral    :lag-rad 0.25     :gain-db -4.5)  ; Lazy arm drag
        (dc-bias         :target :c1-c2-atlas     :offset-rad 0.14  :ceiling-rad 0.01)
        (hard-mute       :target :craniofacial    :velocity-sensitivity 0.0))))


;; =============================================================================
;; 3. TRANSITION AUTOMATION: DRY-TO-WET PHASE-LOCKED MORPH (2-BAR RAMP)
;; =============================================================================
(deftransition morph-dry-to-catwalk
  [:dry-source nominal-biped-gait
   :wet-target (sneaker-sidechain-fx nominal-biped-gait)]

  ;; Wait for zero-crossing so we never crossfade mid-swing
  (sync-trigger
    :wait-for :double-support-zero-crossing
    :lock-zmp-margin-mm 18.0
    :duration-bars 2) ; 4 discrete footsteps to complete transition

  ;; Slew master clock and strip humanize jitter over Bar 1
  (pll-clock-slew
    :curve :hermite-s-curve
    :bpm-ramp [104.0 -> 114.0]
    :swing-ramp [0.12 -> 0.04]
    :jitter-attenuation-db -inf
    :window-bars [0.0 -> 1.0])

  ;; Multi-band kinematic crossover (staggered to prevent ankle clipping)
  (band-split-crossfade
    ;; Band 1 (Top/High-Freq): Kill facial velocity and roll shoulders forward first
    (high-band :nodes [:craniofacial :c1-c2-atlas :c7-t12-thoracic :l4-l5-lumbar]
               :curve :exponential-fast
               :window-bars [0.0 -> 0.75])

    ;; Band 2 (Sub/Coronal): Narrow foot track width only after CoM is centered
    (sub-band  :nodes [:tarsal-coronal :calcaneus :tibiofemoral]
               :curve :equal-power
               :window-bars [0.5 -> 1.5]
               :anti-collision-guard :coronal-bounding-box)

    ;; Band 3 (Mid/Modulation): Bring in the opto-sidechain pump and hip LFO last
    (mid-band  :nodes [:sacroiliac-z :iliac-coronal :glenohumeral]
               :curve :logarithmic-swell
               :window-bars [1.0 -> 2.0]
               :sidechain-threshold-ramp [0.0 -> 1.0])))


;; =============================================================================
;; 4. REAL-TIME EXECUTION STREAM (1 kHz DETERMINISTIC DMA RINGBUFFER)
;; =============================================================================
(defpatch runtime-locomotion-stream
  (compile-to-soa
    :align-bytes 64
    :sample-rate-hz 1000
    :schedule
      [(bars 0 4  nominal-biped-gait)                           ; 4 bars drywalk (backstage)
       (bars 4 6  morph-dry-to-catwalk)                         ; 2 bars phase-locked transition
       (bars 6 32 (sneaker-sidechain-fx nominal-biped-gait))])) ; 26 bars full runway groove
