#ifndef MODEL_OBSERVER_HPP_INCLUDED
#define MODEL_OBSERVER_HPP_INCLUDED

#include <deque>
#include <filesystem>
#include <string>
#include <fstream>
#include "model/model.hpp"

namespace model {

  class Observer
  {
  public:
    virtual ~Observer() {}

    // By default, simply forward the message to the next observer in the chain.
    virtual void notify(long long msg, const class Simulation& sim)
    {
      notify_next(msg, sim);
    }

    void notify_next(long long msg, const class Simulation& sim)
    {
      if (next_) next_->notify(msg, sim);
    }

    void append_observer(Observer* observer)
    {
      if (next_) next_->append_observer(observer);
      else next_ = observer;
    }

    virtual void notify_once(const class Simulation& sim)
    {
      if (next_) next_->notify_once(sim);
    }

  private:
    Observer* next_ = nullptr;
    Observer* parent_ = nullptr;
  };


  class AnalysisObserver : public model::Observer
  {
  public:
    // Constructor reads "output_name" and "sample_freq" from the JSON.
    AnalysisObserver(const std::filesystem::path& out_path, const json& J)
    {
      const std::string out_name = J["output_name"];
      full_out_path_ = (out_path / (out_name + ".csv")).string();

      const float freq_sec = J["sample_freq"];
      // Convert seconds   ticks
      oi_.sample_tick = oi_.sample_freq = static_cast<tick_t>(freq_sec / model::Simulation::dt());

      initialized_ = false;
    }

    virtual ~AnalysisObserver() {}

    struct obs_info
    {
      tick_t sample_freq;  // #ticks between samples
      tick_t sample_tick;  // the next tick at which we call notify_collect
    };


    // Override of Observer::notify(...) to handle PreTick, Tick, Initialized, Finished.
    void notify(long long lmsg, const model::Simulation& sim) override
    {
      using Msg = model::Simulation::Msg;
      auto msg = Msg(lmsg);

      switch (msg) {
        //                                                                     
        // Handle the  initialized  hook once, before any Tick or PreTick.
      case Msg::Initialized: {
        if (!initialized_) {
          notify_init(sim);
          initialized_ = true;
        }
        break;
      }
      case Msg::PreTick: {
        // Called exactly once at the start of every tick.
        notify_pre_tick(sim);

        // If we are about to sample on this tick (i.e., sample_tick), we
        // also invoke notify_pre_collect so that neighbor lists (or anything
        // else) can be forced updated right before we collect.
        if (sim.tick() >= oi_.sample_tick - 1) {
          notify_pre_collect(sim);
        }
        break;
      }

      case Msg::Tick: {
        // Ensure notify_init(...) has been called (in case the simulation
        // never explicitly sent an "Initialized" message before tick=0).
        if (!initialized_ && sim.tick() == 0) {
          notify_init(sim);
          initialized_ = true;
        }

        // At sample time, collect data:
        if (sim.tick() >= oi_.sample_tick) {
          notify_collect(sim);
          oi_.sample_tick = sim.tick() + oi_.sample_freq;
        }

        // If our in memory buffer grows too big, flush it to disk
        if (data_out_.size() > 10000) {
          notify_save(sim);
          data_out_.clear();
        }
        break;
      }

                    //                                                                     
      case Msg::Finished: {
        // If we never called notify_init (because tick never reached 0),
        // do so now.
        if (!initialized_) {
          notify_init(sim);
          initialized_ = true;
        }
        // Write out whatever remains.
        notify_save(sim);
        break;
      }

      default:
        break;
      }

      // Always forward the message down the chain:
      notify_next(lmsg, sim);
    }


  protected:
    // Subclasses may override these hooks as needed:
    virtual void notify_init(const model::Simulation&) {}
    virtual void notify_pre_tick(const model::Simulation&) {}
    virtual void notify_pre_collect(const model::Simulation&) {}
    virtual void notify_collect(const model::Simulation&) {}
    virtual void notify_save(const model::Simulation&) {}

  protected:
    obs_info                      oi_;             // sampling frequency and next sample tick
    bool                          initialized_;    // have we called notify_init yet?
    std::deque<std::vector<float>> data_out_;     // in memory buffer of rows
    std::ofstream                 outfile_stream_;
    std::string                   full_out_path_;
  };

} // namespace model

#endif
