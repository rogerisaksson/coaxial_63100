;; =============================================================================
;; FILE: side_lie.lisp
;; DOMAIN: Bed Ingress on the Side: Edge Perch -> Trochanteric Pendulum along the Bed's Axis
;; =============================================================================
(defpackage :ortho-dsp.side-lie
  (:use :ortho-dsp.core
        :ortho-dsp.transducers
        :ortho-dsp.modulators
        :ortho-dsp.surfaces
        :ortho-dsp.docking
        :ortho-dsp.manifolds
        :ortho-dsp.substrates
        :ortho-dsp.ballistic-handovers
        :ortho-dsp.supervisor)
  (:export #:handover-trochanteric-side-lie
           #:patch-bed-side-lie))
(in-package :ortho-dsp.side-lie)

;; Sat on the bed's edge, her back to it: her trunk levers down onto the near hip while her legs swing
;; up onto the mattress as its counterweight, her pelvis turning the rest of the way to lie along the
;; bed on her side - the trochanter bearing her, no roll onto her back.
(defmaneuver handover-trochanteric-side-lie
  [edge-seated-state &key (mattress-z-mm 540.0) (mattress-eigenfreq-hz 1.80) (bed-axis-yaw-rad 1.5708)]
  (-> edge-seated-state
      (configure-contact-compliance
        :biquad-notch-hz mattress-eigenfreq-hz
        :notch-attenuation-db -18.0)

      (on-contact-threshold
        :sensor-bus :ischial-bilateral
        :trigger-fz-ratio 0.15                  ; her seat bears her from 0.15 of her weight (impedance.PROP_ON)
        :max-impact-velocity-mps 0.02
        (stage-coupled-pendulum
          :total-duration-ms 1550.0
          :fulcrum-node :greater-trochanter-r
          :fulcrum-touchdown-z-mm mattress-z-mm
          :coupled-lever-dynamics
            (:torso-list-slew-rad [0.0 -> 1.5708]       ; her trunk down onto her side
             :pelvic-yaw-slew-rad [0.0 -> bed-axis-yaw-rad] ; turned to lie along the bed
             :leg-pendulum-gain 1.15                   ; her legs up as the counterweight
             :knee-extension-window-ms [600.0 -> 1500.0])
          :contact-wavefront [:greater-trochanter-r]))))

;; Dock at the bed's edge, her back to it; sit on the mattress's edge; the side-lie along it
(defpatch patch-bed-side-lie [approach-gait &key (seat-z-mm 450.0)] ; the bed her eyes see
  (-> approach-gait
      (dock-terminal-stance :standoff-x-mm 85.0 :target-yaw-rad 3.14159)
      (substrate-viscoelastic-mattress :eigenfreq-hz 1.80)
      (transfer-eccentric-handover
        :source-manifold :plantar-bilateral
        :target-manifold :ischial-bilateral
        :descent-duration-ms 1150.0
        :seat-z-mm seat-z-mm)
      (handover-trochanteric-side-lie :mattress-z-mm seat-z-mm)
      (release-impedance-wavefront :terminal-mode :supine-high-z)
      (master-anti-spasm-insert)))
