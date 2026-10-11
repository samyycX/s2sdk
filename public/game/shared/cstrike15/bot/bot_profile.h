#ifndef BOT_PROFILE_H
#define BOT_PROFILE_H

#ifdef _WIN32
#pragma once
#endif

#include "tier0/platform.h"
#include "tier1/strtools.h"
#include "tier1/utllinkedlist.h"
#include "tier1/utlvector.h"

enum BotDifficultyType
{
	BOT_EASY = 0,
	BOT_NORMAL,
	BOT_HARD,
	BOT_EXPERT,
	NUM_DIFFICULTY_LEVELS
};

enum BotProfileTeamType
{
	BOT_TEAM_ANY = 0,
	BOT_TEAM_T = 2,
	BOT_TEAM_CT = 3
};

enum
{
	FirstCustomSkin = 100,
	NumCustomSkins = 100,
	LastCustomSkin = FirstCustomSkin + NumCustomSkins - 1
};

class BotProfileManager;

class BotProfile
{
public:
	BotProfile() = default;
	BotProfile( const BotProfile &src ) { *this = src; }
	BotProfile &operator=( const BotProfile &src );

	const char *GetName() const { return m_name; }
	float GetAggression() const { return m_aggression; }
	float GetSkill() const { return m_skill; }
	float GetTeamwork() const { return m_teamwork; }
	float GetAimFocusInitial() const { return m_aimFocusInitial; }
	float GetAimFocusDecay() const { return m_aimFocusDecay; }
	float GetAimFocusOffsetScale() const { return m_aimFocusOffsetScale; }
	float GetAimFocusInterval() const { return m_aimFocusInterval; }
	uint16 GetWeaponPreference( int i ) const
	{
		Assert( i >= 0 && i < m_weaponPreferenceCount );
		return m_weaponPreference[i];
	}
	int GetWeaponPreferenceCount() const { return m_weaponPreferenceCount; }
	int GetCost() const { return m_cost; }
	int GetSkin() const { return m_skin; }
	bool IsDifficulty( BotDifficultyType diff ) const
	{
		return diff >= BOT_EASY && diff < NUM_DIFFICULTY_LEVELS && ( m_difficultyFlags & ( 1 << diff ) ) != 0;
	}
	int GetVoicePitch() const { return m_voicePitch; }
	float GetReactionTime() const { return m_reactionTime; }
	float GetAttackDelay() const { return m_attackDelay; }
	bool MatchesName( const char *name ) const;
	bool IsValidForTeam( BotProfileTeamType team ) const
	{
		return team == BOT_TEAM_ANY || m_teams == BOT_TEAM_ANY || team == m_teams;
	}
	// AMNOTE: The effect of this preference on weapon behavior is unverified.
	bool PrefersSilencer() const { return m_prefersSilencer; }
	int GetVoiceBank() const { return m_voiceBank; }
	float GetLookAngleMaxAccelNormal() const { return m_lookAngleMaxAccelNormal; }
	float GetLookAngleStiffnessNormal() const { return m_lookAngleStiffnessNormal; }
	float GetLookAngleDampingNormal() const { return m_lookAngleDampingNormal; }
	float GetLookAngleMaxAccelAttacking() const { return m_lookAngleMaxAccelAttacking; }
	float GetLookAngleStiffnessAttacking() const { return m_lookAngleStiffnessAttacking; }
	float GetLookAngleDampingAttacking() const { return m_lookAngleDampingAttacking; }
	const CUtlVector<BotProfile *> &GetTemplates() const { return m_templates; }

protected:
	friend class BotProfileManager;

	// Names and template profiles remain owned by the source, including after copying.
	char *m_name = nullptr;
	float m_aggression = 0.0f;
	float m_skill = 0.0f;
	float m_teamwork = 0.0f;
	float m_aimFocusInitial = 0.0f;
	float m_aimFocusDecay = 1.0f;
	float m_aimFocusOffsetScale = 0.0f;
	float m_aimFocusInterval = 0.5f;
	enum { MAX_WEAPON_PREFS = 16 };
	uint16 m_weaponPreference[MAX_WEAPON_PREFS] = {};
	int m_weaponPreferenceCount = 0;
	int m_cost = 0;
	int m_skin = 0;
	uint8 m_difficultyFlags = 0;
	int m_voicePitch = 100;
	float m_reactionTime = 0.3f;
	float m_attackDelay = 0.0f;
	BotProfileTeamType m_teams = BOT_TEAM_ANY;
	bool m_prefersSilencer = false;
	int m_voiceBank = 0;
	float m_lookAngleMaxAccelNormal = 0.0f;
	float m_lookAngleStiffnessNormal = 0.0f;
	float m_lookAngleDampingNormal = 0.0f;
	float m_lookAngleMaxAccelAttacking = 0.0f;
	float m_lookAngleStiffnessAttacking = 0.0f;
	float m_lookAngleDampingAttacking = 0.0f;
	CUtlVector<BotProfile *> m_templates;
};

typedef CUtlLinkedList<BotProfile *, uint16> BotProfileList;

class BotProfileManager
{
public:
	typedef CUtlVector<char *> VoiceBankList;

	const BotProfile *GetProfile( const char *name, BotProfileTeamType team ) const;
	const BotProfileList *GetProfileList() const { return &m_profileList; }
	const BotProfileList *GetTemplateList() const { return &m_templateList; }
	const VoiceBankList *GetVoiceBanks() const { return &m_voiceBanks; }
	const char *GetCustomSkin( int index ) const
	{
		return index >= FirstCustomSkin && index <= LastCustomSkin ? m_skins[index - FirstCustomSkin] : nullptr;
	}
	const char *GetCustomSkinModelname( int index ) const
	{
		return index >= FirstCustomSkin && index <= LastCustomSkin ? m_skinModelnames[index - FirstCustomSkin] : nullptr;
	}
	const char *GetCustomSkinFname( int index ) const
	{
		return index >= FirstCustomSkin && index <= LastCustomSkin ? m_skinFilenames[index - FirstCustomSkin] : nullptr;
	}

private:
	// The game owns the manager and the profiles it returns.
	BotProfileManager() = delete;
	~BotProfileManager() = delete;
	BotProfileManager( const BotProfileManager & ) = delete;
	BotProfileManager &operator=( const BotProfileManager & ) = delete;

	BotProfileList m_profileList;
	BotProfileList m_templateList;
	VoiceBankList m_voiceBanks;
	char *m_skins[NumCustomSkins];
	char *m_skinModelnames[NumCustomSkins];
	char *m_skinFilenames[NumCustomSkins];
	int m_nextSkin;
};

inline BotProfile &BotProfile::operator=( const BotProfile &src )
{
	if ( this == &src )
		return *this;
	m_name = src.m_name;
	m_aggression = src.m_aggression;
	m_skill = src.m_skill;
	m_teamwork = src.m_teamwork;
	m_aimFocusInitial = src.m_aimFocusInitial;
	m_aimFocusDecay = src.m_aimFocusDecay;
	m_aimFocusOffsetScale = src.m_aimFocusOffsetScale;
	m_aimFocusInterval = src.m_aimFocusInterval;
	for ( int i = 0; i < MAX_WEAPON_PREFS; ++i )
		m_weaponPreference[i] = src.m_weaponPreference[i];
	m_weaponPreferenceCount = src.m_weaponPreferenceCount;
	m_cost = src.m_cost;
	m_skin = src.m_skin;
	m_difficultyFlags = src.m_difficultyFlags;
	m_voicePitch = src.m_voicePitch;
	m_reactionTime = src.m_reactionTime;
	m_attackDelay = src.m_attackDelay;
	m_teams = src.m_teams;
	m_prefersSilencer = src.m_prefersSilencer;
	m_voiceBank = src.m_voiceBank;
	m_lookAngleMaxAccelNormal = src.m_lookAngleMaxAccelNormal;
	m_lookAngleStiffnessNormal = src.m_lookAngleStiffnessNormal;
	m_lookAngleDampingNormal = src.m_lookAngleDampingNormal;
	m_lookAngleMaxAccelAttacking = src.m_lookAngleMaxAccelAttacking;
	m_lookAngleStiffnessAttacking = src.m_lookAngleStiffnessAttacking;
	m_lookAngleDampingAttacking = src.m_lookAngleDampingAttacking;
	m_templates = src.m_templates;
	return *this;
}

inline const BotProfile *BotProfileManager::GetProfile( const char *name, BotProfileTeamType team ) const
{
	if ( name == nullptr )
		return nullptr;
	for ( const BotProfile *profile : m_profileList )
	{
		if ( profile == nullptr || !profile->IsValidForTeam( team ) )
			continue;
		const char *profileName = profile->GetName();
		if ( profileName != nullptr && V_stricmp( profileName, name ) == 0 )
			return profile;
	}
	return nullptr;
}

inline bool BotProfile::MatchesName( const char *name ) const
{
	if ( name == nullptr )
		return false;
	if ( m_name != nullptr )
	{
		const char *profileName = m_name;
		const char *search = name;
		while ( *profileName != '\0' && *search != '\0' )
		{
			uint8 left = static_cast<uint8>( *profileName );
			uint8 right = static_cast<uint8>( *search );
			if ( left >= 'A' && left <= 'Z' )
				left += 'a' - 'A';
			if ( right >= 'A' && right <= 'Z' )
				right += 'a' - 'A';
			if ( left != right )
				break;
			++profileName;
			++search;
		}
		// A '*' matches the remainder of the name.
		if ( *search == '*' || ( *search == '\0' && *profileName == '\0' ) )
			return true;
	}
	for ( const BotProfile *profile : m_templates )
	{
		if ( profile != nullptr && profile->MatchesName( name ) )
			return true;
	}
	return false;
}

#endif // BOT_PROFILE_H
